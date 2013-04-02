require "wo_oo/electronics/intel_hex_grammar"
require "wo_oo/util/hex"
require "pp"

class HexFile
  def self.read(path)
    file_content = File.read(path)
    
    parser = WOoo::Electronics::IntelHexParser.new
    result = parser.parse(file_content)
    
    unless result.nil?
      hex_data = result.eval
      
      WOoo::Util::HexUtil.to_i(hex_data.scan(/../))
    else
      # TO FIX
      puts "PARSING ERROR!"
      puts parser.terminal_failures.join("\n")
      puts "----"
    end
  end
  
  # ----------------------------------------------------------------
  
  def self.write(path, data, start_address = 0)
    next_address = start_address

    File.open(path, "w") do |file|
      while (line_data = data.slice!(0..15)).size > 0
        line_size = line_data.size
        
        file.print ":"
        
        line = WOoo::Util::HexUtil.to_hex8(line_size)
        line += WOoo::Util::HexUtil.to_hex16(next_address)
        line += "00"
        line += WOoo::Util::HexUtil.to_hex8(line_data).join("")
                    
        file.print line
        file.print WOoo::Util::HexUtil.to_hex8(checksum(line))
        
        file.print "\n"

        next_address += line_size
      end
      
      file.print ":00000001FF"
    end
  end
  
  # ----------------------------------------------------------------

  def self.checksum(data)
    checksum = 0

    data.scan(/../).each do |value|
      checksum += WOoo::Util::HexUtil.to_i(value)
    end

    checksum = ((checksum & 255) ^ 255) + 1

    checksum % 256
  end
end
