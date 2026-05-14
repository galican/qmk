# FC220TP

![FC220TP](https://imgur.com/a/7FQYdMS)

A customizable number keypad.

* Keyboard Maintainer: [Keychron](https://www.leopold.co.kr/)
* Hardware Supported: FC220TP
* Hardware Availability: [fc220tp-pd]( https://www.leopold.co.kr/product/fc220tp-pd/248/category/111/display/1/)

Make example for this keyboard (after setting up your build environment):

    make hfdkb/m89u/async:default MULTIMODE_VARIANT=async
    make hfdkb/m89u/sync_a1:default MULTIMODE_VARIANT=sync_a1
    make hfdkb/m89u/sync_a2:default MULTIMODE_VARIANT=sync_a2

Flashing example for this keyboard:

    make hfdkb/m89u/async:default:flash MULTIMODE_VARIANT=async
    make hfdkb/m89u/sync_a1:default:flash MULTIMODE_VARIANT=sync_a1
    make hfdkb/m89u/sync_a2:default:flash MULTIMODE_VARIANT=sync_a2

**Reset Key**: Hold down the key located at *K00*, commonly programmed as *Esc* while plugging in the keyboard.

See the [build environment setup](https://docs.qmk.fm/#/getting_started_build_tools) and the [make instructions](https://docs.qmk.fm/#/getting_started_make_guide) for more information. Brand new to QMK? Start with our [Complete Newbs Guide](https://docs.qmk.fm/#/newbs).
