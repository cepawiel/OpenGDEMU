# Address Map for MCU

## 0x00
### Bit 0 : ? (RO)
0 = Primary
1 = Secondary

### Bit 1 : ? (RW)
Set if command is pending

## 0x02
### Bit 0 : ERR  (RW)
### Bit 3 : DRQ  (RW)
### Bit 5 : DF   (RW)
### Bit 6 : DRDY (RW)
### Bit 7 : BSY  (RW)


## 0x04
### Bits 7 to 0 : DEVICE (RW)

## 0x06
### Bits 7 to 0 : ABRT (RW)

## 0x08
### Bits 7 to 0 : COMMAND (RW)
### Bits 15 to 8 : Features (RO)
?? I added for debugging ??

## 0x0A
### Bits 15 to 0 : SectorCount (RW)

## 0x0C
### Bits 7 to 0 : LBALow (RW)
### Bits 15 to 8 : LBAMid (RW)

## 0x0E 
### Bits 7 to 0 : LBAHigh (RW)
### Bits 15 to 8 : LBALowPrev (RW)

## 0x10
### Bits 7 to 0 : LBAMidPrev (RW)
### Bits 15 to 8 : LBAHighPrev (RW)

## 0x12
### Bits 15 to 8 : FIFOs (RW)

## 0x14
### Bits 10 to 0 : Read FIFO Count (RW)

## 0x16
### Bits 10 to 0 : Write FIFO Count

## 0x18
### Bits 15 to 0 : Read Test (RO)