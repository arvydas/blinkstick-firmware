#!/usr/bin/ruby -w

require "wo_oo/util/hex"
require "./intel_hex.rb"

#get the current serial number
file = File.new("serial.txt", "r")
line = file.gets 
file.close 

#increment it
serial_number = line.to_i + 1

#build serial number string
serial_string = "BS%06d-1.0" % serial_number

puts serial_string

data = []

#this byte is required for oscillator
data.push 0

#fill in the serial number
serial_string.split("").each do |c|
  data.push c.ord
end

#fill the rest with zeroes
while data.length < 512 do
  data.push 0
end

#write the hex file with Intel format
HexFile.write("eeprom.hex", data)

#write the new serial number
file = File.new("serial.txt", "w")
file.puts serial_number
file.close
