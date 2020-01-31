# Arduino Si5351 Library tuned for size and click noise free. #

This library is tuned for size on the Arduino platform, it will control CLK0, CLK1 and CLK2 outputs for the Si5351A (the version with just 3 clocks out).

But there is not such thing as free lunch: please read the __Two of three__ section below; and sure it's click free with some provisions, keep reading.

## Inspiration ##

This work is based on the previous work of these great people:

* [Etherkit/NT7S:](https://github.com/etherkit/Si5351Arduino) The mainstream full featured lib, with big code as well (based on Linux kernel code)
* [QRP Labs demo code from Hans Summers:](http://qrp-labs.com/synth/si5351ademo.html) The smallest and simple ones on the net.
* [DK7IH demo code:](https://radiotransmitter.wordpress.com/category/si5351a/) The first clickless noise code on the wild.
* [Jerry Gaffke integer routines for the Raduino and ubitx](https://github.com/afarhan/ubitx.git)

## Features ##

This are so far the implemented features (Any particular wish? use the Issues tab for that):

* **NEW:** All integer math now, that make us save ~1k of firmware space (_Worst induced error is below +/- 2 Hz_)
* **NEW:** You has a way to verify the status of a particular clock (_Enabled/Disabled by the Si.clkOn[clk] var_)
* Custom XTAL passing on init (Default is 27.000 MHz, see _Si.init()_ )
* You can pass a correction to the xtal while running (See _Si.correction()_ )
* You have a fast way to power off all outputs of the Chip. (See _Si.off()_ )
* You can enable/disable any output at any time (By default all outputs are off after the init procedure, you has to enable them)
* You can only have 2 of the 3 outputs running at any moment, see "Two of three" below.
* It's free of click noise while you move on frequency, yes, there is a catch, see "Click noise free" below.
* Power control on each output independently (See _setPower(clk, level)_ on the lib header, initial default is to lowest level: 2mA)
* You don't need to include and configure the Wire (I2C) library, this lib do that for you already.
* Frequency limits are not hard coded on the lib, so you can stress your hardware to it's particular limit (_You can move usually from ~3kHz to ~225 MHz, far away from the 8kHz to 160 MHz limits from the datasheet_)

## Click noise free ##

The click noise while tunning the chip came from the following actions (stated in the datasheet & app notes):

```
1 - Turn CLKx output off.
2 - Update the PLL and MultySynth registers
3 - Reset the PLLx for the new calculations.
4 - Turn CLKx output on.
```

In my code I follow a strategy of just do that at the start of the freq output and move the PLLs and multisynths freely without reseting the PLLs or multisynths outputs if not needed.

_**Note:** in version 0.3 I found a signal level problem between frequencies beyond 112 MHz._

_For the way we handle the chip it needs a reset() (yes, click noise included!) for every frequency change beyond VCO/8 (starting around 112 MHz) or you will get a strange level variation in the output of the Chip._

_That was fixed in version 0.4, Please note that this is not a problem if you use it below 112 MHz, never the less we included a failsafe trigger that resets the PLL every 10 kHz if below VCO/8. this is not noticeable in my test with a homebrew Bitx 40v3 (SMD)_

_The arbitrary 10 kHz spacing was chosen from comments in the Bitx20 mail-list from the QRP Labs guys, thanks for the tip._

## The start sequence is important ##

Yes, in your setup code segment you must initialize it in the following sequence:

* Initialize the library with the default or optional Xtal Clock.
* Apply correction factor (if needed)
* Set sweet spots frequencies to **both** clock outputs.
* Force a reset of the PLLs.
* Enable the desired outputs.

Here you have an example code of what I mean ("Si" is the lib instance):

```
setup() {
    (... code here ...)

    //////////////////////////////////
    //        Si5351 functions       /
    //////////////////////////////////

    // Init the library, in this case with the defaults
    Si.init();

    // Optional, apply my calculated correction factor
    Si.correction(-1250);

    // set some sweet spot freqs
    Si.setFreq(0, 25000000);       // CLK0 output
    Si.setFreq(1, 145000000);      // CLK1 output

    // force the first reset
    Si.reset();

    // enable only the needed outputs
    Si.enable(0);
    Si.enable(1);

    (... other code here ...)
}

```



If you need to apply/vary the correction factor **after** the setup process you will get a click noise on the next setFreq() to apply the changes.

## Two of three ##

Yes, there is a tittle catch here with CLK1 and CLK2: both share PLL_B and as we use math to produce an integer division and very low jitter (aka: _phase noise_) you can only use one of them at a time.

Note: _In practice you can, but the other will move from the frequency you set, which is an unexpected behavior, so I made them mutually exclusive._

This are the valid combinations for independent clocks output.

* CLK0 and CLK1
* CLK0 and CLK2

Again: You can't use CLK1 and CLK2 at the same time, as soon as you set one of them the other will shut off. That's why you get two of three and one of them must be always CLK0.

## Author & contributors ##

The only author is Pavel Milanes, CO7WT, a cuban amateur radio operator; reachable at pavelmc@gmail.com, Until now I have no contributors or sponsors.

## Where to download the latest version? ##

Always download the latest version from the [github repository](https://github.com/pavelmc/Si5351mcu/)

See ChangeLog.md and Version files on this repository to know what are the latest changes and versions.

## If you like to give thanks... ##

No payment of whatsoever is required to use this code: this is [Free/Libre Software](https://en.wikipedia.org/wiki/Software_Libre), nevertheless donations are very welcomed.

I live in Cuba island and the Internet/Cell is very expensive here (USD $1.00/hour), you can donate anonymously internet time or cell phone air time to me via [Ding Topups](https://www.ding.com/) to keep me connected and developing for the homebrewers community.

If you like to do so, please go to Ding, select Cuba, select Cubacell (for phone top up) or Nauta (for Internet time)

* For phone topup use this number (My cell, feel free to call me if you like): +53 538-478-19
* For internet time use this user (Nauta service): co7wt@nauta.com.cu (that's not an email but an user account name)

Thanks!
