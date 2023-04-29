---
title: How to Copy a MicroSD
status: hot
author: Pito Salas
---
# Copy MicroSD!

* Current best resource on web: [The fastest way to clone an SD card on macOS](https://blog.jaimyn.dev/the-fastest-way-to-clone-sd-card-macos/)

## Identify your sd card
You’ll need to find out which disk your SD card represents. You can run `diskutil list` and should see an output like below:
```
$ diskutil list

/dev/disk1 (synthesized):
   #:                       TYPE NAME                    SIZE       IDENTIFIER
   0:      APFS Container Scheme -                      +500.0 GB   disk1
                                 Physical Store disk0s2
   1:                APFS Volume Macintosh HD — Data     396.0 GB   disk1s1
   2:                APFS Volume Preboot                 81.9 MB    disk1s2
   3:                APFS Volume Recovery                528.5 MB   disk1s3
   4:                APFS Volume VM                      4.3 GB     disk1s4
   5:                APFS Volume Macintosh HD            11.0 GB    disk1s5

/dev/disk4 (external, physical):
   #:                       TYPE NAME                    SIZE       IDENTIFIER
   0:     FDisk_partition_scheme                        *31.9 GB    disk4
   1:             Windows_FAT_32 boot                    268.4 MB   disk4s1
   2:                      Linux                         31.6 GB    disk4s2
```
From that output we can see that our SD card must be `/dev/disk4` as our card is 32GB in size and has a fat32 and linux partition (standard for most raspberry pi images). You should add an r in front of disk4 so it looks like this `/dev/rdisk4`. The r means when we’re copying, it will use the “raw” disk. For an operation like this, it is much more efficient.

## Copy the SD card as a disk image (dmg)

Now you should run the following command, replacing 4 with whatever number you identified as your sd card:

`sudo gdd if=/dev/rdisk4 of=sd_backup.dmg status=progress bs=16M`

Tip: you can experiment with different numbers for the block size by replacing bs=16M with larger or smaller numbers to see if it makes a difference to the speed. I’ve found 16M the best for my hardware.

You should see some progress feedback telling you the transfer speed. If you’d like to experiment with different block sizes, just type ctrl + c to cancel the command, then you can run it again.

Once the command has finished running, you’ll end up with a file in your home directory called sd_backup.dmg. If you’d like to backup multiple SD cards (or keep multiple backups!) simply replace sd_backup.dmg with a different file name. This will contain a complete disk image of your SD card. If you’d like to restore it, or clone it to another SD card, read on.

## Copy the disk image (dmg) to your SD card

You’ll first need to unmount your SD card. Do not click the eject button in finder, but run this command, replacing 4 with whatever number you identified as your sd card 

`sudo diskutil unmountDisk /dev/disk4`

Then to copy the image, run the following command:

`sudo gdd of=/dev/rdisk4 if=sd_backup.dmg status=progress bs=16M`


Tip: you can experiment with different numbers for the block size by replacing bs=16M with larger or smaller numbers to see if it makes a difference to the speed. I’ve found 16M the best for my hardware.

You should see some progress feedback telling you the transfer speed. If you’d like to experiment with different block sizes, just type ctrl + c to cancel the command, then you can run it again.

Once the command has finished running, your SD card should be an exact copy of the disk image you specified.





