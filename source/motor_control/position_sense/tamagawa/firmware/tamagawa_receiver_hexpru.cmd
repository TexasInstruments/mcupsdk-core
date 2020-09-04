-b
-map=tamagawa_receiver_hex.map
-image

ROMS {
                PAGE 0:
                text: o = 0x0, l = 0x1000, files={text.bin}
                PAGE 1:
                data: o = 0x0, l = 0x1000, files={data.bin}
}
