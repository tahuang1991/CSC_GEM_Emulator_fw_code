setMode -bs
setMode -bs
setMode -bs
setMode -bs
setCable -port auto
Identify -inferir 
identifyMPM 
assignFile -p 1 -file "/home/cscdev/20150206/CSC_GEM_Emulator_fw_code/work/CSC_GEM_Emulator.bit"
Program -p 1 
ReadIdcode -p 1 
ReadUsercode -p 1 
ReadUsercode -p 1 
assignFile -p 1 -file "/home/cscdev/20150206/CSC_GEM_Emulator_fw_code/work/CSC_GEM_Emulator.bit"
Program -p 1 
assignFile -p 1 -file "/home/cscdev/20150206/CSC_GEM_Emulator_fw_code/work/CSC_GEM_Emulator.bit"
Program -p 1 
setMode -bs
setMode -ss
setMode -sm
setMode -hw140
setMode -spi
setMode -acecf
setMode -acempm
setMode -pff
setMode -bs
setMode -ss
setMode -sm
setMode -hw140
setMode -spi
setMode -acecf
setMode -acempm
setMode -pff
setMode -bs
setMode -bs
setMode -bs
setCable -port auto
Identify -inferir 
identifyMPM 
ReadUsercode -p 1 
attachflash -position 1 -bpi "XCF128X"
assignfiletoattachedflash -position 1 -file "/home/cscdev/XilinxProj/GLIB/GLIB_Flash.mcs"
Program -p 1 -dataWidth 16 -rs1 NONE -rs0 NONE -bpionly -e -loadfpga 
ReadUsercode -p 1 
ReadUsercode -p 1 
Identify -inferir 
identifyMPM 
assignFile -p 1 -file "/home/cscdev/20150206/CSC_GEM_Emulator_fw_code/work/CSC_GEM_Emulator.bit"
ReadIdcode -p 1 
ReadUsercode -p 1 
Program -p 1 
assignFile -p 1 -file "/home/cscdev/20150206/CSC_GEM_Emulator_fw_code/work/CSC_GEM_Emulator_previous.bit"
Program -p 1 
assignFile -p 1 -file "/home/cscdev/20150206/CSC_GEM_Emulator_fw_code/work/CSC_GEM_Emulator.bit"
Program -p 1 
assignFile -p 1 -file "/home/cscdev/20150206/CSC_GEM_Emulator_fw_code/work/CSC_GEM_Emulator_previous.bit"
Program -p 1 
ReadIdcode -p 1 
ReadIdcode -p 1 
assignFile -p 1 -file "/home/cscdev/20150206/CSC_GEM_Emulator_fw_code/work/CSC_GEM_Emulator.bit"
Program -p 1 
ReadUsercode -p 1 
assignFile -p 1 -file "/home/cscdev/20150206/CSC_GEM_Emulator_fw_code/work/CSC_GEM_Emulator.bit"
Program -p 1 
ReadUsercode -p 1 
Program -p 1 
ReadUsercode -p 1 
Program -p 1 
ReadUsercode -p 1 
Program -p 1 
ReadUsercode -p 1 
Program -p 1 
ReadUsercode -p 1 
ReadUsercode -p 2 
ReadUsercode -p 3 
setMode -pff
setMode -pff
addConfigDevice  -name "EmulatorBoard" -path "/home/cscdev/XilinxProj/CSC_GEM_Emulator_fw_code/work"
setSubmode -pffversion
setAttribute -configdevice -attr multibootBpiType -value ""
addDesign -version 0 -name "0"
setAttribute -configdevice -attr compressed -value "FALSE"
setAttribute -configdevice -attr compressed -value "FALSE"
setAttribute -configdevice -attr autoSize -value "FALSE"
setAttribute -configdevice -attr fileFormat -value "mcs"
setAttribute -configdevice -attr fillValue -value "FF"
setAttribute -configdevice -attr swapBit -value "FALSE"
setAttribute -configdevice -attr dir -value "UP"
setAttribute -configdevice -attr multiboot -value "FALSE"
setAttribute -configdevice -attr multiboot -value "FALSE"
setAttribute -configdevice -attr spiSelected -value "FALSE"
setAttribute -configdevice -attr spiSelected -value "FALSE"
addPromDevice -p 1 -size 0 -name xcf32p
addPromDevice -p 2 -size 0 -name xcf32p
setMode -pff
setMode -pff
setMode -pff
setSubmode -pffversion
setMode -pff
addDeviceChain -index 0
setMode -pff
addDeviceChain -index 0
setMode -pff
setSubmode -pffversion
setMode -pff
setMode -pff
addDeviceChain -index 0
setAttribute -design -attr name -value "0000"
addPromDevice -p 2 -size 0 -name xcf32p
deletePromDevice -position 1
addPromDevice -p 3 -size 0 -name xcf32p
deletePromDevice -position 2
setMode -bs
setMode -bs
setMode -bs
setMode -pff
setMode -pff
setMode -pff
setMode -pff
addDevice -p 1 -file "/home/cscdev/XilinxProj/CSC_GEM_Emulator_fw_code/work/CSC_GEM_Emulator-fast.bit"
setMode -pff
setSubmode -pffversion
generate
setCurrentDesign -version 0
setMode -bs
setMode -bs
setMode -bs
assignFile -p 1 -file "/home/cscdev/XilinxProj/CSC_GEM_Emulator_fw_code/work/CSC_GEM_Emulator-fast.bit"
ReadUsercode -p 2 
assignFile -p 2 -file "/home/cscdev/XilinxProj/CSC_GEM_Emulator_fw_code/work/EmulatorBoard_1.mcs"
setAttribute -position 2 -attr packageName -value ""
ReadUsercode -p 3 
assignFile -p 3 -file "/home/cscdev/XilinxProj/CSC_GEM_Emulator_fw_code/work/EmulatorBoard_0.mcs"
setAttribute -position 3 -attr packageName -value ""
setMode -bs
setMode -bs
setMode -ss
setMode -sm
setMode -hw140
setMode -spi
setMode -acecf
setMode -acempm
setMode -pff
setMode -bs
saveProjectFile -file "/home/cscdev/XilinxProj/CSC_GEM_Emulator_fw_code/work/EmulatorBoardDownload.ipf"
Program -p 2 -e -parallel -u 150316c1 -ver 0 customercode:150316c1cf32 -defaultVersion 0 
ReadUsercode -p 3 -u 150316c0 
ReadCustomerCode -p 3 
ReadUsercode -p 3 -u 150316c0 
Program -p 3 -e -parallel -u 150316c0 -ver 0 customercode:150316c0cf32 -defaultVersion 0 
ReadUsercode -p 3 -u 150316c0 
ReadCustomerCode -p 2 
ReadUsercode -p 1 
setMode -bs
setMode -bs
setMode -ss
setMode -sm
setMode -hw140
setMode -spi
setMode -acecf
setMode -acempm
setMode -pff
setMode -bs
saveProjectFile -file "/home/cscdev/XilinxProj/CSC_GEM_Emulator_fw_code/work/EmulatorBoardDownload.ipf"
setMode -bs
setMode -bs
setMode -ss
setMode -sm
setMode -hw140
setMode -spi
setMode -acecf
setMode -acempm
setMode -pff
setMode -bs
saveProjectFile -file "/home/cscdev/XilinxProj/CSC_GEM_Emulator_fw_code/work/EmulatorBoardDownload.ipf"
setMode -bs
setMode -pff
setMode -bs
deleteDevice -position 1
deleteDevice -position 1
deleteDevice -position 1
setMode -bs
setMode -ss
setMode -sm
setMode -hw140
setMode -spi
setMode -acecf
setMode -acempm
setMode -pff
