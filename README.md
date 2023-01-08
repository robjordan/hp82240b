# hp82240b
Emulate this vintage IR printer, used with Hewlett Packard calculators.
Currently text-only, and tested with HP-17BII.

![Here's a photo](https://photos.app.goo.gl/MMHFgfQ2cndNFict5).

Connect TX of ATTINY1614 via USB/Serial adapter, then run:
minicom -b 115200 -8 -R utf-8 -D /dev/ttyUSB1


