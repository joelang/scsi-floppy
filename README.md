# scsi-floppy
SASI/SCSI-1 to floppy disk interface

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
;	Copyright 2017,2018 Joseph C. Lang
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
; The monitor is a modified version of Keymon3 (Cornell University)
; I don't have any rights to redistribute, so It's not included here
; and commented out.
