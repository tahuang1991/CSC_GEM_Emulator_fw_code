## CSC_GEM_Emulator_fw_code  GEMCSC2016Feb

CSC_GEM_Emulator_fw_code is developed to read or write/send GEM/CSC data from/to PC,and to OTMB through fibers  



This version of GEMCSC emulator originated from TAMU development back to 2014 and later Andrew simplified the code by removing the obsolete code. The code is more human-readable after Andrew's changes


The code is from https://github.com/andrewpeck/emulator_fw/tree/testattamu

### fibers in CSC_GEM_Emulator_fw_code
* fiber0 --> GEM fiber1 (counting from 1), 56 bits (4 clusters) per clock
* fiber1 --> DCFEB 1 (counting from 1) 48 bits(8bits per layer and 6 layers per fiber) per clock
* fiber2 --> DCFEB 2
* fiber3 --> DCFEB 3
* fiber4 --> DCFEB 4
* fiber5 --> DCFEB 5 
* fiber6 --> DCFEB 6 
* fiber7 --> DCFEB 7 

### version control
 2019 March 24, TXDIFFCTRL is changed from 4'b0111 into 4'b1110 to increase differential signal amplitude as firefly in new OTMB may be more sensitive to it. Tao

### commands

```
    parameter CMD_WRITE   = 16'hf7f7; // writes to block ram
    parameter CMD_READ    = 16'hf3f3; // reads from block ram
    parameter CMD_DUMP    = 16'hfdfd; // dump block ram contents

    // not used in sw
    parameter CMD_SEND    = 16'hfefe;
    parameter CMD_REWIND  = 16'hf0f0;

    parameter EOF         = 16'hf7fd; // 16'b1111011111111101
```




## functionalities
 
### Function1
```
""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Building CLCT patterns
 clock:
      ck40, negedge ck40
 led: 
     
 input:   // variables to init and control this function
      clct_pattern. control which patterns to be built 
 output:  //variables as results of this function 
      triad_word_l*, encode clct patterns
      
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
```



### Function2
```
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Block RAM read and write control
 clock:
       gbe_txclk2, for bother reading and writing 
 led:
 
 input:
     inputs from RAMB36E1
 output:
     outputs from RAMB36E1

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
```



### Function3
```
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
GigaBit Ethernet Transceiver
  clock:
       inputs in submodule.

  led:
   
  input:

  output:

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
```



### Function4

```
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Sending out patterns to TMB
  clock:
   
  led:

  input: 7 DCFEB fibers and 1 GEM fibers. 7 DCFEB fibers coded comparator digis from 4DCFEBs. 1 GEM fibers coded 4 GEM clusters


  output: data transmitted to OTMB


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
```
