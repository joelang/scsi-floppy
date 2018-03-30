# scsi-floppy
SASI/SCSI-1 to floppy disk interface


SCSI-Floppy:
 This project allows you to use floppy
 disk drives  on a SCSI/SASI controller

 This implementation is minimal SASI/SCSI-1

 The intent is to allow a SASI/SCSI interface 
 (something like a DTC-10 or xebec)
 on a S-100 crate to use Floppy disks without
 an available S-100 floppy controler
 this allows a smaller cbios and removes most
 timing constraints on the host
