# hp82240b
Emulate this vintage IR printer, used with Hewlett Packard calculators.
Currently text-only, and tested with HP-17BII.

IR receiver extracted from [HP C4013A](http://www.colin99.co.uk/extras/irda/hp_c4103a_irda.htm), and connected with an ATTINY1614.

![Here's a photo](./img/PXL_20230108_184011698.jpg)

Connect TX of ATTINY1614 via USB/Serial adapter, then run:
```
minicom -b 115200 -8 -R utf-8 -D /dev/ttyUSB1
```
Useful links:
- [HP Roman character set](https://en.wikipedia.org/wiki/HP_Roman)
- [HP 82240B character set](https://hwiegman.home.xs4all.nl/fileformats/hp82240b/ptrcodes.htm)
- [Search for Unicode character codes](https://unicode-table.com/en/)
- [Martin Hepperle's adaptation of C4113A](http://www.mh-aerotools.de/hp/red-eye/HP-IR%20Receiver%20with%20Arduino.pdf)
- [HP82240B Technical Interfacing Guide](https://www.keesvandersanden.nl/calculators/downloads/82240bte.pdf)
