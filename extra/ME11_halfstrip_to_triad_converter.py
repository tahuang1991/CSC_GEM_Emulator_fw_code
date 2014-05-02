def halfstrip_to_triad (hs):
  if 0 <= hs <= 31: # ME1/1a - 64 strips (128 halfstrips) per 4 DCFEBs
    dcfeb = 0
    first_hs_in_dcfeb = 0
  elif 32 <= hs <= 63:
    dcfeb = 1
    first_hs_in_dcfeb = 32
  elif 64 <= hs <= 95:
    dcfeb = 2
    first_hs_in_dcfeb = 64
  elif 96 <= hs <= 127:
    dcfeb = 3
    first_hs_in_dcfeb = 96
  elif 128 <= hs <= 159:
    dcfeb = 4
    first_hs_in_dcfeb = 128
  elif 160 <= hs <= 191:
    dcfeb = 5
    first_hs_in_dcfeb = 160
  elif 192 <= hs <= 224:
    dcfeb = 6
    first_hs_in_dcfeb = 192
  else:
    raise Exception("Error #1: wrong halfstrip number in chamber")
  
  hs_in_dcfeb = hs - first_hs_in_dcfeb
  
  if 0 <= hs_in_dcfeb < 4:
    distrip = 1
    first_hs_in_distrip = 0
  elif 4<= hs_in_dcfeb <8:
    distrip = 10
    first_hs_in_distrip = 4
  elif 8<= hs_in_dcfeb <12:
    distrip = 100
    first_hs_in_distrip = 8
  elif 12<= hs_in_dcfeb <16:
    distrip = 1000
    first_hs_in_distrip = 12
  elif 16<= hs_in_dcfeb <20:
    distrip = 10000
    first_hs_in_distrip = 16
  elif 20<= hs_in_dcfeb <24:
    distrip = 100000
    first_hs_in_distrip = 20
  elif 24<= hs_in_dcfeb <28:
    distrip = 1000000
    first_hs_in_distrip = 24
  elif 28<= hs_in_dcfeb <32:
    distrip = 10000000
    first_hs_in_distrip = 28
  else:
    raise Exception("Error #2: wrong halfstrip number in DCFEB")
  
  hs_in_distrip = hs_in_dcfeb - first_hs_in_distrip
  
  if hs_in_distrip == 0 or hs_in_distrip == 1:
    strip = 0
    if hs_in_distrip == 0:
      halfstrip = 0
    elif hs_in_distrip == 1:
      halfstrip = 1
  elif hs_in_distrip == 2 or hs_in_distrip == 3:
    strip = 1
    if hs_in_distrip == 2:
      halfstrip = 0
    elif hs_in_distrip == 3:
      halfstrip = 1
  
  strip = strip*distrip
  halfstrip = halfstrip*distrip
  
  print hs, "=>", dcfeb, "(DCFEBS: 0-6)", distrip, strip, halfstrip


halfstrip_to_triad(99)
halfstrip_to_triad(58)
halfstrip_to_triad(85)
