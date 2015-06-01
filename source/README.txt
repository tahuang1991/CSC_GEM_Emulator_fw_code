


  CSC_GEM_Emulator:

 
Function1
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



Function2
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



Function3
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
GigaBit Ethernet Transceiver
  clock:
       inputs in submodule.

  led:
   
  input:

  output:

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""



Function4
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Sending out patterns to TMB
  clock:
   
  led:

  input:


  output:


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""

