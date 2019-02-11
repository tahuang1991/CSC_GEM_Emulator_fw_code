## CSC_GEM_Emulator_fw_code  GEMCSC2016Sep

CSC_GEM_Emulator_fw_code is developed to read or write/send GEM/CSC data from/to PC,and to OTMB through fibers  



This version of GEMCSC emulator originated from TAMU development back to 2014 and later Andrew continued to work on it by removing obsolete code and adding GEM features like GEM cluster packer, GEM fibers


The code is from https://github.com/andrewpeck/emulator_fw/tree/master

### fibers in CSC_GEM_Emulator_fw_code
* fiber0 --> GEM fiber1 (counting from 1)
* fiber1 --> DCFEB 1 (counting from 1)
* fiber2 --> DCFEB 2
* fiber3 --> DCFEB 3
* fiber4 --> DCFEB 4
* fiber5 --> GEM fiber2 
* fiber6 --> GEM fiber3 
* fiber7 --> GEM fiber4 

### commands

```
    parameter CMD_WRITE   = 16'hf7f7; // writes to block ram
    parameter CMD_READ    = 16'hf3f3; // reads from block ram
    parameter CMD_DUMP    = 16'hfdfd; // dump block ram contents
    parameter CMD_PACK    = 16'hf1f1; // fill block ram through cluster packer

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
Pack Vfat Sbits into GEM clusters, 24 VFATs to 8 cluster per chamber
  clock:
   
  led:

  input:  24 vfat Sbits which are stored in DRAM before packing


  output: 8 GEM cluster per GEM layer

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
```



### Function5

```
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Sending out patterns to TMB
  clock:
   
  led:

  input: 4 DCFEB fibers and 4 GEM fibers. 4 DCFEB fibers coded comparator digis from 4DCFEBs. 4 GEM fibers coded GEM clusters from GE1/1


  output: data transmitted to OTMB


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
```

