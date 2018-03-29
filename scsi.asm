;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;SCSI-Floppy:
; This project allows you to use floppy
; disk drives  on a SCSI/SASI controller
;
; This implementation is minimal SASI/SCSI-1
;
; The intent is to allow a SASI/SCSI interface 
; (something like a DTC-10 or xebec)
; on a S-100 crate to use Floppy disks without
; an available S-100 floppy controler
; this allows a smaller cbios and removes most
; timing constraints on the host
;
; Nothing in the hardware design should prevent 
; implementing higher SCSI (2 or 3) compatability
; I don't have a need for that. (yet?)
;
; CPU:  ATmega32A 16Mhz clock
; SCSI: NCR5380/DP8490
; FDC:  WD1772 8Mhz clock
;
;	Copyright 2017 Joseph C. Lang
;		Released to the public domain
;
; Edit history:
;	4/2/2017  original code entry
;	4/12/2017 misc edits to make AVRA happy
;	5/30/2017 edit/add some comments
;	7/19/2017 fixed LBN2CHS
;	9/14/2017 started testing on real hardware
;		  serial debug monitor runs
;		  added more comments
;		  added "call" to keymon to help debug 
;	2/15/2018 fixed swapped bits port c bits
;	2/23/2018 added setup delay (16 us.) after all
;		  writes to FD command register
;		  read of PC 720k formatted disks now works
;	3/28/2018 added delay after write trackto allow time
;                 for tunnel erase to complete
;
; Supports 4 drives
; MFM 250Kbit/s
; double side
; 80 track
; 9 sectors
; (IBM 720K format)
;
;AVR atmega32a fuses (avrdude syntax):
;-U lfuse:w:0xe0:m -U hfuse:w:0xd9:m
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; commented out for Atmel studio
;.include "m32def.inc"
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	.CSEG
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;port A: bidirectional data bus for external peripherals
;
;port B: output
;	drive select 0-3
;	side 4
;	density 5
;	not used 6
;	reset 7 
;
;port C: output
;	external peripheral address 0-3
;	seop 5
;	read/-wr 6
;	strobe 7
;
;decoded by U6 to produce:
;	ior- iow- dmaack- scsi_cs- fdc_cs- eop-
;
;port D: serial (SIO keymon debug)
;	 inputs
;		SCSI DRQ 4
;		SCSI IRQ 5
;		FDC DRQ  6
;		FDC IRQ  7
;
;
;floppy disk parameters:
.equ TRACKS	= 80
.equ SPT	= 9	;sectors per track
.equ SIDES	= 2	;sides
.equ MAXLBN	= 1439	;last LBN number
;
;scsi select enable:(SC_R4)
.equ SC_TID	= 1	;my SCSI target ID
;I need to fix this to use jumpers
;
;SCSI MODE REGISTER:(SC_R2)
;uncomment one:
.equ SC_MODE	= 0x42	;target mode + DMA
;.equ SC_MODE	= 0x72	;target mode + DMA + parity
;This should be jumpers too
.equ SC_NDMA	= 0xfd	;negate dma mode bit
;
;
;SCSI ICR:(SC_R1) (initiator command reg)
.equ SC_BUSY	= 0x08	;assert busy bit
.equ SC_OUT	= 0x01	;assert SCSI data bus
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;floppy disk register addresses: (WD1772 address)
; port C bit patterns
.equ FD_CMD	= 0x00	;FDC command reg (WR)
.equ FD_STAT	= 0x00	;FDC status reg (RD)
.equ FD_TRK	= 0x01	;FDC current track (RW)
.equ FD_SCTR	= 0x02 	;FDC desired sector (RW)
.equ FD_DATA	= 0x03	;FDC data register (RW)
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;SCSI controller register addresses:
;
; port C bit patterns
.equ SC_R0	= 0x08	;NCR 5380 register 0
.equ SC_R1	= 0x09
.equ SC_R2	= 0x0a
.equ SC_R3	= 0x0b
.equ SC_R4	= 0x0c
.equ SC_R5	= 0x0d
.equ SC_R6	= 0x0e
.equ SC_R7	= 0x0f	;through register 7
.equ SC_DMAK	= 0x07	;5380 DMA ack address
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;control port (port C) bit patterns:
;these bits are set/reset to control access
;to the FDC and SCSI
.equ BT_READ	= 6	;read/write- bit
.equ BT_EOP	= 5	;end of process (to 5380)
.equ BT_STROB	= 7	;enable for decode and eop

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;drive select bit patterns: (port B)
.equ DR_0	= 0x01	;drive 0
.equ DR_1	= 0x02	;drive 1
.equ DR_2	= 0x04	;drive 2
.equ DR_3	= 0x08	;drive 3
.equ DR_SIDE	= 0x10	;side 1
.equ DR_SNGL	= 0x20	;single density (not used)
;not used 0x40
.equ DR_NORST	= 0x80	;reset out (act low)
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;floppy status bits (port D)
.equ FD_IRQ	= 7	;FDC IRQ
.equ FD_DRQ	= 6	;FDC DRQ
;
;scsi status bits (port D)
.equ SC_IRQ	= 5	;SCSI IRQ
.equ SC_DRQ	= 4	;SCSI DRQ
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;SCSI phase bits:
;
;this is a little confusing when coding 
;a target. The scsi spec is written 
;from the POV of the initiator
;so in and out seem backwards
;
.equ PH_DATO	= 0x00  ;data from initiator
.equ PH_DATI	= 0x01  ;data to initiator
.equ PH_CMD	= 0x02  ;cmd from initiator
.equ PH_STAT	= 0x03  ;status (cmd) to initiator
.equ PH_MESO	= 0x06  ;message from initiator
.equ PH_MESI	= 0x07  ;message to initiator
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;supported SCSI commands:
;
.equ CM_SENSE	= 0x03	;request sense
.equ CM_READ	= 0x08	;read
.equ CM_WRITE	= 0x0a	;write
.equ CM_CHECK	= 0x00	;check drive ready
.equ CM_WTRACK	= 0x06	;write track
.equ CM_FORMAT	= 0x04	;format drive
.equ CM_RECAL	= 0x01	;recalibrate (seek zero)
.equ CM_SEEK	= 0x0b	;seek to track
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;scsi error responses
;
.equ ER_OK	= 0x00	;no error
.equ ER_DNR	= 0x04	;drive not ready
.equ ER_NT0	= 0x06	;no track zero
.equ ER_SNF	= 0x94	;sector not found
.equ ER_CRC	= 0x9e	;CRC error
.equ ER_CMD	= 0x20	;illegal command
.equ ER_IDA	= 0xb1	;illegal disk address
.equ ER_FMT	= 0x17	;formatting error
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;wd1772 commands
;
.equ FC_RECAL	= 0x00	;seek track zero
.equ FC_SEEK	= 0x10	;seek to track
.equ FC_STEPI	= 0x50	;step in 1 cyl
.equ FC_READ	= 0x80	;read sector
.equ FC_WRITE	= 0xa2	;write sector
.equ FC_FMT	= 0xf6	;write track
.equ FC_FIRQ	= 0xd0	;force interrupt
.equ FC_PRE	= 0xfd	;precomp enable
.equ FC_SETL	= 0x04	;settle delay enable
;
;wd1772 error masks
.equ FC_WRERR	= 0x5d	;bits for write error
.equ FC_RDERR	= 0x1d	;bits for read error
.equ FC_FMERR	= 0x4d	;bits for write track error
.equ FC_TRK0	= 0x04	;bit for track 0 flag
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;register variables/usage:
;
.def LSTERR	= R0	;last floppy error (debug)
; R15 LUN save
; R16 acca
; R17 count/bits
; R18 temp
.def DRIVE	= R19	;drive select bit pattern
.def TRACK	= R20	;desired track num
.def SECTOR	= R21	;desired sector num
.def SIDE	= R22	;desired side
.def RETRY	= R23	;error retry bit pattern
.def LUN	= R24	;current LUN (drive)
.def SETTLE	= R25	;add head settle delay
;data pointer R26,R27	;pointer to SRAM

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;sram 
	.DSEG
	.ORG 0x100
;
CDB:	.byte 10	;command block
SENSE:	.byte 4		;sense data
TRKTAB:	.byte 4		;drive track table
BUFFER:	.byte 512	;sector/block buffer
;
	.CSEG
	.ORG 0
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;jump vectors
;reset/interrupt vectors
	jmp	scsi_go ;1 reset
	jmp	dummy	;2 int0
	jmp	dummy	;3 int1
	jmp	dummy	;4 int2 
	jmp	dummy	;5 timer2 comp
	jmp	dummy	;6 timer2 ovf
	jmp	dummy	;7 timer1 capt
	jmp	dummy	;8 timer1 compa
	jmp	dummy	;9 timer1 compb
	jmp	dummy	;10 t1 ovf
	jmp	dummy	;11 timer0 comp
	jmp	dummy	;12 timer0 ovf
	jmp	dummy	;13 spi
;
;debug monitor is commented out
;	jmp	monitor ;14 usart rxc
	jmp	dummy   ;14 usart rxc
;
	jmp	dummy	;15 usart udre
;
;debug monitor is commented out
;	jmp	monitor ;16 usart txc
 	jmp	dummy   ;16 usart txc
;
	jmp	dummy	;17 adc
	jmp	dummy	;18 ee rdy
	jmp	dummy	;19 ana comp 
	jmp	dummy	;20 twi
	jmp	dummy	;21 spm rdy
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;unexpected IRQ lands here
dummy:  nop		;for debugging
here:	jmp	here	;fatal error
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;program cold start
;
scsi_go:
;
;reset stack
	cli			;disable IRQ
	ldi r16,low(RAMEND)	;set stack
	out SPL,r16
	ldi r16,high(RAMEND)
	out SPH,r16
;
;debug monitor is commented out
;	call RXinit		;init serial monitor
;
;reset ports
	clr R16
	out DDRA,R16		;set A to input (data bus)
	out DDRD,R16		;set D to input (irq/drq)
	out PORTC,R16		;output 0 to port C (address)
;
;select drive 0
	ldi R16,DR_0
	mov DRIVE,R16		;set current drive
	out PORTB,R16		;select drive 0 w/reset asserted
	clr LUN			;set LUN to zero
;
	ldi R16,0xFF
	out DDRB,R16		;set B to output (drive sel)
	out DDRC,R16		;set C to output
;
	ldi R17,100		;time delay reset active
	call delay		;kill time
;
;release FDC and SCSI reset
	ldi R16,DR_0+DR_NORST
	out PORTB,R16		;sel drive 0 remove reset
;
;zero out (most of) RAM
;also provide delay for end of reset
zero_buf:
	ldi r26,low(CDB)	;point to RAM
	ldi r27,high(CDB)
	ldi r16,0		;fill byte
	ldi r17,0		;count
zero_b0:
	st X+,r16		;clear data
	st X+,r16
	st X+,r16
	dec r17			;dec word count
	brne zero_b0		;loop till done
;
;init SCSI chip
	ldi R16,SC_MODE		;set target mode
	ldi R17,SC_R2		;mode register
	call put_reg		;write register
;
;wait for SCSI bus reset to clear (if active)
rst_act:
	ldi R17,SC_R1
	call get_reg		;get status
	andi R16,0x80		;mask to reset bit
	brne rst_act		;loop while active
;
;wait for floppy controller to idle
wait:
	ldi R17,FD_STAT
	call get_reg
	andi R16,0x01		;loop while busy
	brne wait
 	rjmp  setup_sel		;wait for SCSI select
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;All commands return here
;R16 = status code
;put status in sense block
;send status, send end message
;then disconnect
;
snd_stat:
	sts SENSE,R16		;save status in sense
	ldi R16,PH_STAT		;set SCSI phase
	ldi R17,SC_R3		;to status in
	call put_reg
;
;create error response byte
	lds R17,CDB+1		;get LBA/LUN
	andi R17,0xe0		;mask to LUN
	lds R16,SENSE		;recover status
	tst R16			;error set?
	mov R16,R17		;mov LUN to response
	breq snd_s01		;br if no error
	ori R16,0x02		;set error bit in resp.
;
;send status
snd_s01:
	call sdma_out1		;send ending status
;
;send end message
	ldi R16,PH_MESI		;set SCSI phase
	ldi R17,SC_R3		;to message in
	call put_reg
	clr R16
	call sdma_out1		;send message=00 (done)
	clr R16
	ldi R17,SC_R3
	call put_reg		;clear bus phase
	clr R16
	ldi R17,SC_R1
	call put_reg		;disconnect
;
;command complete 
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;setup for SCSI select
;reset stack
;setup 5380 mode and target ID
;
setup_sel:
	cli			;stop interrupts
	ldi r16,low(RAMEND)	;reset stack
	out SPL,r16
	ldi r16,high(RAMEND)
	out SPH,r16
;
	ldi R17,FD_STAT
	call get_reg		;reset FDC IRQ (if any)
;
	ldi R17,SC_R7
	call get_reg		;reset SCSI IRQ (if any)
	sei			;allow interrupts
;
	ldi R16,SC_MODE
	ldi R17,SC_R2
	call put_reg		;set target mode
;
	ldi R16,SC_TID
	ldi R17,SC_R4
	call put_reg		;set target ID
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;wait for SCSI select.
;test FDC motor on bit and clear
;drive select when motor stops
;poll for SCSI IRQ bit
;
WSloop:
 	ldi R17,FD_STAT
	call get_reg		;get FDC status
 	andi R16,0x80		;motor on bit
	brne WSmon		;br if motor on
;
;motor not on so clear drive sel port
	ldi R16,DR_NORST
	out PORTB,R16		;clear drive select
;
;test for scsi IRQ loop if not
WSmon:		
	sbis PIND,SC_IRQ	;IRQ active?
	rjmp WSloop		;loop if not set
;
;IRQ is active so validate selection
;check for error or reset
	ldi R17,SC_R5
	call get_reg
	andi R16,0x20		;parity error?
	brne fatal
	ldi R17,SC_R4
	call get_reg
	andi R16,0x02		;select error?
	breq fatal
;
;check for more than two bits active
	ldi R17,SC_R0		;get scsi data
	call get_reg
	ldi R17,8		;shift count
	clr R0			;clear active bits
nextbt:
	ror R16			;rotate sel bits
	brcc zerobt		;skip carry clear
	inc R0			;count bit
zerobt:
	dec R17			;dec loop count
	brne nextbt		;no more bits?
;
	dec R0			;1 is good
	breq svalid
	dec R0			;2 is good
	breq svalid
	rjmp setup_sel		;no good try again
;
;valid select so set busy
svalid:
	ldi R16,SC_BUSY
	ldi R17,SC_R1
	call put_reg		;assert busy bit
;
;wait for sel to negate
sel_act:
	ldi R17,SC_R1
	call get_reg		;get status
	andi R16,0x04		;mask to sel bit
	brne sel_act
;
;clear select enable 
	ldi R17,SC_R4
	ldi R16,0x00		;clear select enable	
	call put_reg
;	
	ldi R17,SC_R7
	call get_reg		;reset irq 
	rjmp get_cdb		;go get command
;
;parity error or reset so start over
fatal:
	rjmp scsi_go		;start over
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I am selected so get command block from host
;
get_cdb:
	ldi R16,PH_CMD		;set SCSI phase
	ldi R17,SC_R3		;to cmd out
	call put_reg
;
	ldi R26,low(CDB)	;set X pointer
	ldi R27,high(CDB)
	ldi R17,6		;set byte count
 	call sdma_in		;get CDB
;
;vaildate LUN
;only LUN 0-3 supported
	lds R16,CDB+1		;get LUN from CDB
	tst R16
	brmi bad_lun		;LUN>3?
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;decode command
;
	lds R16,CDB		;get cmd from CDB
	cpi R16,CM_SENSE
	brne notSN
	rjmp req_sense
notSN:
	cpi R16,CM_READ
	brne notRD
	rjmp rd_blks
notRD:
	cpi R16,CM_WRITE
	brne notWR
	rjmp wr_blks
notWR:
	cpi R16,CM_CHECK
	brne notCK
	rjmp chk_rdy
notCK:
	cpi R16,CM_WTRACK
	brne notWT
	rjmp wr_track
notWT:
	cpi R16,CM_FORMAT
	brne notFM
	rjmp format
notFM:
	cpi R16,CM_RECAL
	brne notRE
	rjmp recal
notRE:
	cpi R16,CM_SEEK
	brne bad_cmd
	rjmp cyl_seek
;
;bad command if you get here
bad_cmd:
	ldi R16,ER_CMD		;invalid command
	rjmp snd_stat
;
;LUN>3
bad_lun:
	ldi R16,ER_DNR		;drive not ready
	rjmp snd_stat
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;command: send sense data
;send 4 bytes of sense data
;followed by status
req_sense:
	ldi R16,PH_DATI		;set SCSI phase
	ldi R17,SC_R3
	call put_reg
	ldi R26,low(SENSE)	;set pointer
	ldi R27,high(SENSE)
 	ldi R17,4		;set byte count
	call sdma_out		;send sense data
	clr R16			;can't have an err
	rjmp snd_stat		;send status
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;command:check drive ready
;this is a stub since I dont have a ready line
;if the LUN is valid the drive is ready
chk_rdy:
	call lbn2sense		;copy LBN to sense
	call sel_drv		;select drive
	ldi R16,0		;ok return
	rjmp snd_stat		;send completion
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;command: read blocks from disk
;
rd_blks:
	call lbn2sense		;copy LBN to sense
	call sel_drv		;select drive
	ldi R16,PH_DATI		;set SCSI phase
	ldi R17,SC_R3		;to data in (to host)
	call put_reg
;
rloop:
	call fd_read		;read floppy
	brne rbk_err		;floppy error?
	ldi R26,low(BUFFER)	;point to buffer
	ldi R27,high(BUFFER)
 	clr R17			;set byte count=256
	call  sdma_out		;send data to host
 	clr R17			;set byte count=256
	call  sdma_out		;send data to host
;
	lds R16,CDB+4		;get block count
	dec R16
	sts CDB+4,R16		;put back
	tst R16			;last block?
	breq rbk_done
	call  inc_lbn		;next block
	rjmp rloop		;loop
;
rbk_err:
	ldi R16,ER_CRC		;set error code
rbk_done:
	rjmp snd_stat

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;command: write blocks to disk
;
wr_blks:
	call lbn2sense		;copy LBN to sense
	call sel_drv		;select drive
	ldi R16,PH_DATO		;set SCSI phase
	ldi R17,SC_R3
	call put_reg
wloop:
	ldi R26,low(BUFFER)	;point X to buffer
	ldi R27,high(BUFFER)
 	clr R17			;set byte count=256
	call sdma_in		;get data block
 	clr R17			;set byte count=256
	call sdma_in		;get data block
	call fd_write		;write to floppy
	brne wbk_err		;floppy error?
;
	lds R16,CDB+4		;get block count
	dec R16			;dec count
	sts CDB+4,R16		;put back
	tst R16			;last block?
	breq wbk_done		;then exit
;
	call  inc_lbn		;next block
	rjmp wloop
;
wbk_err:
	ldi R16,ER_SNF		;set error code
wbk_done:
	rjmp snd_stat
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;command: format drive
;format tracks starting at LBN (track)
;
format:
	call lbn2sense		;copy LBN (data ignored)
	call sel_drv		;select drive
;
floop: 
	call wrt_trk		;write one track
	brne fmt_err		;exit if error
;
;check for side 1 track 79
	cpi SIDE,SIDES-1
	brne finctr			;
	cpi TRACK,TRACKS-1	;last one?
	breq fmt_done
;
;inc track and loop
finctr:
	call inc_trk
	rjmp floop		;and write track
;
fmt_done:
	clr r16			;good status
	rjmp snd_stat
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;command: write single track
;
wr_track:
	call lbn2sense		;copy LBN to sense
	call sel_drv		;select drive
	call wrt_trk		;write track
	breq fmt_done		;no error?
;
fmt_err:
	ldi R16,ER_FMT		;error status
	rjmp snd_stat
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;command: recalibrate
;select drive and recal heads to track zero
;
recal:
	call lbn2sense		;fill in sense data
	call lbn2chs		;set up CHS
	call sel_drv		;select drive
	call seek_zero		;recal to trk 0
	ldi R17,FD_STAT
	call get_reg		;get status
	andi R16,FC_TRK0	;mask to trk 0 bit
	ldi R16,ER_NT0		;error status
	breq seek_err
seek_ok:
	clr R16			;no error status
seek_err:
	rjmp snd_stat
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;command: seek
;select drive and seek to track
;
cyl_seek:
	call lbn2sense		;fill in sense data
	call lbn2chs		;set up CHS
	call sel_drv		;select drive
	call seek2trk		;seek to target trk
	clr R16			;no error
	rjmp snd_stat
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: step in
;step one track towards spindle
;
step_in:
	ldi R16,FC_STEPI	;fdc step in cmd
	rjmp do_fcmd
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: recal head to track zero
;
seek_zero: 
	ldi R16,FC_RECAL	;fdc seek zero cmd
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: issue floppy command
;and wait for done
;set settle delay flag
;R16 command
;
do_fcmd:
	ldi SETTLE,FC_SETL	;set settle delay flag
	ldi R17, FD_CMD
	call put_reg		;send cmd to FDC
	ldi R17,25
	call delay		;cmd setup delay
do_wait:
	sbis PIND,FD_IRQ
	rjmp do_wait		;loop till IRQ set
no_wait:
	ldi R17,FD_STAT
	call get_reg		;get status
	ret

;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: seek_to_track
;skip seek if already on track
;
seek2trk:
	clr SETTLE		;preset no settle delay
	ldi R17,FD_TRK
	call get_reg		;get current track
	cp R16,TRACK		;same as requested?
	breq no_wait		;skip seek if same
;
	mov R16,TRACK		;get target track
	ldi R17,FD_DATA
	call put_reg		;send track to FDC
	ldi R16,FC_SEEK		;fdc seek cmd
	rjmp do_fcmd		;do floppy cmd
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: read single sector with retries
;LBN in sense data,drive selected
;exit: R16 is error status
;
fd_read:
	ldi RETRY,0x91		;set retry pattern
	call lbn2chs		;convert LBN to CHS
	call set_side		;select side
;
rd_rtry:
	call seek2trk		;seek to track
;
;set sector number
	mov R16,SECTOR		;get requested sctr
	ldi R17,FD_SCTR
	call put_reg		;put in controller
;
;reset data pointer
	ldi R26,low(BUFFER)	;set pointer to buffer
	ldi R27,high(BUFFER)
;
;issue read command
	ldi R16,FC_READ
	or  R16,SETTLE		;set delay flag
	ldi R17,FD_CMD
	call put_reg
	ldi R17, 25		;delay 16 us.
	call delay
;set data port and  direction
	ldi R17,FD_DATA		;select floppy data
	ldi R16,0xff
	out PORTA,R16		;pullups on
	ldi R16,0x00
	out DDRA,R16		;direction=in
	out PORTC,R17		;select reg
	sbi PORTC,BT_READ	;assert read
	rjmp rdloop
;
;DRQ is active so handle read data request
rd_drqa:
	sbi PORTC,BT_STROB	;assert strobe
	nop			;delay for read pulse
	nop			;width
	nop
	nop
	in R16,PINA		;read bus
	cbi PORTC,BT_STROB	;remove strobe
	st X+,R16
;
rdloop:
	sbic PIND,FD_DRQ	;DRQ set?
	rjmp rd_drqa		;if fdrq get byte
	sbis PIND,FD_IRQ	;IRQ set?
	rjmp rdloop		;loop if no IRQ/DRQ
;
;IRQ is active so check ending status
rd_irqa:
	ldi R17,FD_STAT
	call get_reg		;get status
	mov LSTERR,R16		;save error for debug
	andi R16,FC_RDERR	;mask to read error bits
	breq rd_done		;none set then good read
	lsr RETRY		;check for retry
	breq rd_errx		;zero none left
	brcc rd_rtry		;carry not set try again
	call seek_zero		;recal position
	rjmp rd_rtry
;
rd_errx:
	tst R16			;status=NZ
rd_done:
	ret
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;souroutine:write single sector with retries
;sense data=LBN,drive already selected
;exit: R16 is error status
;
fd_write:
	ldi RETRY,0x91  	;set retry pattern
	call lbn2chs		;convert LBN 2 CHS
	call set_side		;select side
wr_rtry:
	call seek2trk		;seek to track
;
;set sector number
	mov R16,SECTOR
	ldi R17,FD_SCTR
	call put_reg
;
;reset data pointer 
	ldi R26,low(BUFFER)	;set pointer to data
	ldi R27,high(BUFFER)
;
;issue write command
	ldi R16,FC_WRITE	;FDC write cmd
	or R16,SETTLE		;set settle delay flag
	cpi TRACK,43		;track >43?
	brcc wr_go
	andi R16,FC_PRE		;then set precomp en
wr_go:
	ldi R17,FD_CMD
	call put_reg		;issue command
	ldi R17,25
	call delay
;
;set data port and  direction
	ldi R17,FD_DATA 	;select floppy data
	ldi R16,0xFF
	out DDRA,R16		;direction=out
	out PORTC,R17		;select reg
	rjmp wrloop
;
;DRQ active so handle write data request
wr_drqa:
	ld R16,X+		;get data byte
	out PORTA,R16		;send data to bus
	sbi PORTC,BT_STROB	;assert strobe
	nop			;delay for read pulse
	nop			;width
	nop
	nop
	cbi PORTC,BT_STROB	;remove strobe
wrloop:
	sbic PIND,FD_DRQ	;DRQ set?
	rjmp wr_drqa		;if fdrq get byte
	sbis PIND,FD_IRQ	;IRQ set?
	rjmp wrloop		;loop if no IRQ/DRQ
;
;IRQ active so check ending status
wr_irqa:
	ldi R17,FD_STAT
	call get_reg		;get status
	mov LSTERR,R16		;save error
	andi R16,FC_WRERR	;mask to write error bits
	breq wr_done		;none set then good read
	lsr RETRY		;check for retry
	breq wr_errx		;zero none left
	brcc wr_rtry		;carry not set try again
	call seek_zero		;recal position
	rjmp wr_rtry
;
wr_errx:
	tst R16			;status=NZ
wr_done:
;
;delay for trim erase not needed here since 
;SCSI I/O will prevent the heads from moving too soon
;
	ret
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: convert LBN to CHS
;logical block num must be set in sense block
lbn2chs:
	clr TRACK		;clear track num
	clr SIDE		;clear side
	lds R30,SENSE+3		;get LBN
	lds R31,SENSE+2
incloop:
	sbiw R30,SPT		;sub sectors per trk
	brcs ufl		;underflow?
 	inc TRACK		;inc track
	rjmp incloop		;loop
ufl:
	adiw R30,SPT		;fix underflow
	inc R30			;sectors start at 1
	mov SECTOR,R30		;set desired sector
	clc
	ror TRACK		;lsb of track is side
	brcc side0
	inc SIDE		;side to 1
side0:
	ret
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: add 1 to LBN (in sense data)
;and check (doesn't return if too big)
inc_lbn:
	lds R16,SENSE+3		;get LBN
	lds R17,SENSE+2
	inc R16			;inc block
	brne inc_done
	inc R17			;propagate carry
inc_done:
	sts SENSE+3,R16		;put LBN back
	sts SENSE+2,R17
;
	subi R16,low(MAXLBN+1)	;is LBN too big
	sbci R17,high(MAXLBN+1)
	brsh lbn_err
	ret
;
lbn_err:
	ldi R16,ER_IDA		;illegal address
	rjmp snd_stat		;terminate cmd
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;advance LBN to next track
;
inc_trk:
	ldi R17,SPT			;get sectors per track
	lds R16,SENSE+3		;get LBN
	add R16,R17			;add
	sts SENSE+3,R16		;put back
	ldi R17,0
	lds R16,SENSE+2		;
	adc R16,R17			;add carry
	sts SENSE+2,R16		;put back
	ret
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: get_reg
;get byte from external device register
;R16 data
;R17 device select bits
get_reg:
	ldi R16,0x00		;direction=in
	out DDRA,R16		;set port A DDR
	ldi R16,0xff		;turn on pullups
	out PORTA,R16
	out PORTC,R17		;set reg
	sbi PORTC,BT_READ	;assert read
	sbi PORTC,BT_STROB	;assert strobe
	nop			;delay for read pulse
	nop			;width
	nop
	nop
	in R16,PINA		;read bus
	cbi PORTC,BT_STROB	;remove strobe
	ret
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: put_reg
;put byte to external device register
;R16 data
;R17 device select bits
put_reg:
	out PORTC,R17		;select reg
	out PORTA,R16		;set data bus
	ldi R16,0xFF		;direction=out
	out DDRA,R16		;set port A DDR
	sbi PORTC,BT_STROB	;assert strobe
	nop			;delay for write pulse
	nop			;width
	nop
	nop
	cbi PORTC,BT_STROB	;remove strobe
	ret
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: mk_ptr
;make pointer to entry in track table TRKTAB[lun]
;LUN must be set
mk_ptr:
	mov R16,LUN		;get LUN
	ldi r28,low(TRKTAB)	;point to RAM
	ldi r29,high(TRKTAB)
	add R28,R16		;add LUN to base
	brcc mkpxit
	inc R29
mkpxit:
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: select drive
;if same LUN just set select bits and return
;else save current FDC track in track table
;get track for new LUN and put in FDC
;set drive select bits
sel_drv:
;
;get requested LUN
	lds R16,CDB+1		;get LUN
	rol R16			;shift LUN to 
	rol R16			;bits 0-2
	rol R16
	rol R16
	andi R16,7		;mask to LUN bits
;
;if same as current LUN skip FDC register update
	cp R16,LUN		;same as last time?
	mov R15,R16		;save LUN
	breq set_sel		;just set the sel bits
;
;save current track from FDC
	call mk_ptr		;point to track table
	ldi R17,FD_TRK		;get track register
	call get_reg
	st Y,R16		;save it
;
;restore track position for new LUN
	mov LUN,R15		;set new LUN
	call mk_ptr		;point to table
	ld R16,Y		;get track
	ldi R17,FD_TRK
	call put_reg		;send to controller
;
;convert LUN to drive select bits
	clr R16			;clear sel
	sec			;preset sel bit
sel_shft:
	rol R16			;shift into bits
	dec R15			;dec LUN
	brpl sel_shft		;keep shifting till underflow
	mov DRIVE,R16		;save select bit
;
;set drive select bits in port
set_sel:
	mov R16,DRIVE
	ori R16,DR_NORST	;not a reset
	out PORTB,R16
	ret
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: set_side
;set side select bit
set_side:
	mov R16,DRIVE		;get drive bits
	ori R16,DR_NORST	;no reset
	tst SIDE		;requested side
	breq setport
	ori R16,DR_SIDE		;set side 1 bit
setport:
	out PORTB,R16		;set drv sel port
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; subroutine: rst_dma
; clear dma enable bit
rst_dma:
	push R16
	push R17
	ldi R17,SC_R2		;mode register
	ldi R16,SC_MODE		;
	andi R16,SC_NDMA	;reset dma bit
	call put_reg
	pop R17
	pop R16
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: setup registers for DMA in
;
set_dmai:
	push R16
	push R17
;
;set SCSI data direction to input
	ldi R17,SC_R1
	ldi R16,SC_BUSY		;negate direcion bit
	call put_reg		;dont drive scsi data
;
;set dma mode
	ldi R17,SC_R2
	ldi R16,SC_MODE
	call put_reg		;turn on dma mode
;
;start DMA
	ldi R17,SC_R6
	call put_reg		;trigger DMA start
;
;set port A direction
	ldi R16,0x00
	out DDRA,R16		;direction=in
	ldi R16,0xff		;pullups on
	out PORTA,R16
;
;point to pseudo dma register
	ldi R17,SC_DMAK
	out PORTC,R17
	sbi PORTC,BT_READ	;read external buss
;
	pop R17
	pop R16
	ret
;
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: get 1 byte with EOP set
;R16 data byte
sdma_in1:
	call set_dmai		;setup for DMA input
;
;secondary entry point for last byte
;of multi byte transfer
sdma_in2:
	sbi PORTC,BT_EOP	;assert EOP
;
;wait for DRQ or IRQ to set
sdmai_wt:
	sbic PIND,SC_DRQ
	rjmp dmai_by		;dreq set
	sbis PIND,SC_IRQ
	rjmp sdmai_wt		;loop
;
;scsi IRQ set something bad happened
; command can't be completed so quit
	call rst_dma		;turn off dma
	rjmp scsi_go		;start over
;
;SCSI drq set so get last byte and return
dmai_by:
	sbi PORTC,BT_STROB	;assert strobe
	nop			;delay for read pulse
	nop
	nop
	nop
	in R16,PINA		;get byte
	cbi PORTC,BT_STROB	;negate strobe
	st X+,R16		;store byte
	call rst_dma	;reset dma mode
	ret
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: sdma_in
;get block from initiator
;end request with EOP
;R17=byte count
;R27,R26 (X) data pointer
;scsi phase already set
sdma_in:
	call set_dmai		;setup for DMA input
sdi_lp: 
	dec R17 
	breq sdma_in2		;get last byte
;
;wait for DRQ or IRQ to set
sdi_wt:
	sbic PIND,SC_DRQ
	rjmp sdi_by		;dreq set
	sbis PIND,SC_IRQ
	rjmp sdi_wt		;loop IRQ not set
;
;scsi IRQ set something bad happened
; can't complete command so quit
	call rst_dma		;turn off dma
	rjmp scsi_go		;start over
;
;get data byte and loop
sdi_by:
	sbi PORTC,BT_STROB	;assert strobe
	nop			;delay for read
	nop			;pulse width
	nop
	nop
	in R16,PINA		;get byte
	cbi PORTC,BT_STROB	;negate strobe
	st X+,r16		;store it
	rjmp sdi_lp		;loop
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: set_dmao
;setup registers for DMA out
set_dmao:
	push R16
	push R17
;
;set SCSI data direction out
	ldi R17,SC_R1
	ldi R16,SC_BUSY+SC_OUT
	call put_reg		;drive scsi data  bus
;
;set dma mode
	ldi R17,SC_R2
	ldi R16,SC_MODE
	call put_reg		;enable DMA mode
;
;start DMAs
	ldi R17,SC_R5
	ldi R16,0x00
	call put_reg		;trigger DMA start
;
;point to pseudo dma register
	ldi R17,SC_DMAK
	out PORTC,R17
;
;set port A direction
	ldi R16,0xFF
	out DDRA,R16		;direction=out
;
	pop R17
	pop R16
	ret
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: send last byte
;send 1 byte with EOP set
;R16 data byte
;scsi phase already set
sdma_out1: 
	call set_dmao		;setup for dma out
;
;secondary entry point for last byte
;of multi byte transfer
sdma_out2:
	sbi PORTC,BT_EOP	;assert EOP
	out PORTA,R16		;put data on bus
;
;wait for DRQ or IRQ to set
sdmao_wt:
	sbic PIND,SC_DRQ
	rjmp dmao_by		;DRQ set?
	sbis PIND,SC_IRQ
	rjmp sdmao_wt		;loop IRQ not set
;
;scsi IRQ set something bad happened
	call rst_dma		;turn off dma
	rjmp scsi_go		;start over
;
;SCSI drq set
dmao_by:
	sbi PORTC,BT_STROB	;assert strobe
	nop
	nop
	nop
	nop
	cbi PORTC,BT_STROB	;negate strobe
	ldi R17,1
	call delay
;
;wait for req/ack idle
;
;test for req active
waitidl:
	ldi R17,SC_R4
	call get_reg		;get bus status
	andi R16,0x20		;req bit
	brne waitidl
;
;test for ack active
	ldi R17,SC_R5
	call get_reg
	andi R16,0x01		;ack bit
	brne waitidl
;
;stop driving bus
	call rst_dma		;turn off dma mode
	ldi R17,SC_R1
	ldi R16,SC_BUSY
	call put_reg		;stop driving bus
	ret
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: sdma_out
;send block to initiator
;end request with EOP set
;R17 byte count
;R27,R26 (X) data pointer
;scsi phase already set
sdma_out:
	call set_dmao		;set up for DMA out
sdo_lp:
	ld R16,X+		;get data
	dec R17 
	breq sdma_out2		;send last byte
	out PORTA,R16		;send byte
;
;wait for DRQ or IRQ to set
sdo_wt:
	sbic PIND,SC_DRQ
	rjmp sdo_by		;DRQ set
	sbis PIND,SC_IRQ
	rjmp sdo_wt		;IRQ not set
;
;scsi IRQ set so something bad happened
	rjmp scsi_go		;start over
;
;send data byte
;data is already on port A
sdo_by:
	sbi PORTC,BT_STROB	;assert strobe
	nop
	nop
	nop
	nop
	cbi PORTC,BT_STROB	;negate strobe
	rjmp sdo_lp
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: copy lbn to sense
;doesn't return if LBN too large
lbn2sense:
	lds R18,CDB+1		;copy 3 bytes
	sts SENSE+1,R18		;of address
	lds R17,CDB+2
	sts SENSE+2,R17
	lds R16,CDB+3
	sts SENSE+3,R16
;
;check LBN
	andi R18,0x1f		;MSB must be zero
	brne too_big		;too big error
	subi R16,low(MAXLBN+1)	;cmp to max
	sbci R17,high(MAXLBN+1)
	brsh too_big		;too big error

	ret			;ok exit
;
too_big:
	ldi R16,ER_IDA		;illegal address
	rjmp snd_stat		;error exit
	
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: write track
;drive should be selected and on desired track
;
wrt_trk:
	call lbn2chs		;convert to c,h,s
	call set_side		;select side
	call seek2trk		;seek to desired track
	ldi SECTOR,9		;sector count
	ldi R30,low(secilv<<1)	;sector interleave pointer
	ldi R31,high(secilv<<1)	;times 2
	ldi R16,FC_FMT		;FDC write trk cmd
	cpi TRACK,43		;precomp needed?
	brcc fmt_go
	andi R16,FC_PRE		;set precomp enable
fmt_go:
	ldi R17,FD_CMD
	call put_reg		;start FDC write trk
	ldi R17,25		;delay 16 us.
	call delay
;
;set bus register and direction
	ldi R17,FD_DATA		;select floppy data register
	out PORTC,R17		;
	ldi R16,0xFF
	out DDRA,R16		;direction=out
;
;send track data pattern
;index address mark
WRiam:
	ldi R16,0x4e		;data pattern $4e
	ldi R17,80		;count	80
	call writeM		;write multiple bytes
;
	ldi R16,0		;data pattern $00 
	ldi R17,12		;count 12
	call writeM		;write multiple bytes
;
	ldi R16,0xf6
	ldi R17,3
	call writeM
;
	ldi R16,0xfc
	ldi R17,1
	call writeM
;
;per sector data
WRsctr:
	ldi R16,0x4e		;sector gap
	ldi R17,50
	call writeM
;
	ldi R16,00
	ldi R17,12
	call writeM
;
	ldi R16,0xf5
	ldi R17,3
	call writeM
;
	ldi R16,0xfe		;preset CRC
	ldi R17,1
	call writeM
;
	mov R16,TRACK		;write track num
	ldi R17,1
	call writeM
;
	mov R16,SIDE		;write side
	ldi R17,1
	call writeM
;
	lpm R16,Z+		;get sector num
	ldi R17,1
	call writeM
;
	ldi R16,02		;write sector size
	ldi R17,1
	call writeM
;
	ldi R16,0xf7		;write header CRC
	ldi R17,1
	call writeM
;
	ldi R16,0x4e		;write data gap
	ldi R17,22
	call writeM
;
	ldi R16,0x00
	ldi R17,12
	call writeM
;
	ldi R16,0xf5
	ldi R17,3
	call writeM
;
	ldi R16,0xfb		;data ID
	ldi R17,1
	call writeM
;
	ldi R16,0xe5		;write data
	ldi R17,0
	call writeM
;
	ldi R16,0xe5		;write data
	ldi R17,0
	call writeM
;
	ldi R16,0xf7		;write CRC
	ldi R17,1
	call writeM
;
	ldi R16,0x4e		;write gap
	ldi R17,40
	call writeM
;
	dec SECTOR		;dec sector count
	breq wr_fill
	rjmp WRsctr		;loop if not zero
;
;fill the rest of the track with 0x4e
;ends with IRQ active
wr_fill:
	ldi R16,0x4e
	call writei		;write till FDC IRQ
;
;return with ending status
	ldi R17,FD_STAT
	call get_reg		;get ending status and clear IRQ
	clr R17				;delay for trim erase
	call delay
	call delay
	call delay
	call delay
	andi R16,FC_FMERR	;mask to write error bits
	ret
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: write byte multiple times to floppy.
;wait for floppy DRQ/IRQ then assert strobe
;loop till R17=0 or IRQ set
;port direction and reg sel bits must already be set
;R16 byte to write
;R17 repeat count
;
writeM:
	out PORTA,R16		;put data on bus
wmloop:
	sbic PIND,FD_DRQ
	rjmp wmbyte		;DRQ set
	sbis PIND,FD_IRQ
	rjmp wmloop		;loop IRQ not set
;
;not expecting IRQ (error) here so fatal error
;return up one level with error set
;this will terminate the format
wmerr:
	pop R16			;discard return address
	pop R16
	ldi R16,ER_FMT		;set error code 
	tst R16			;set not equal
	ret			;error exit
;
;write byte and loop
wmbyte:
	sbi PORTC,BT_STROB	;assert strobe
	nop			;delay for write pulse
	nop			;width
	nop
	nop
	cbi PORTC,BT_STROB	;remove strobe
	dec R17			;dec loop count
	brne wmloop
	ret			;exit
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: write byte multiple times until
; FDC asserts IRQ. Wait for DRQ/IRQ
;port direction and reg sel bits must already be set
;R16 byte to write
;
writeI:
	out PORTA,R16		;put byte on bus
wiloop:
	sbic PIND,FD_DRQ
	rjmp wibyte		;DRQ set
	sbis PIND,FD_IRQ
	rjmp wiloop		;loop IRQ not set
;
wiexit:
	ret			;else its IRQ 
;
wibyte:
	sbi PORTC,7		;assert strobe
	nop			;delay for write pulse
	nop			;width
	nop
	nop
	cbi PORTC,7		;remove strobe
	rjmp wiloop
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subroutine: delay
;R17 delay count
;
delay:
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	dec R17
	brne delay
	ret
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;sector interleave table
;
secilv:
	.db 	1,4
	.db 	7,2
	.db 	5,8
	.db 	3,6
	.db 	9,0
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;debug monitor is commented out
;.include "keymon3.inc"
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


