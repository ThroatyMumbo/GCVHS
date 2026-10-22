    .section .data.field_wave, "aw"
    .balign 4
    .global field_wave
field_wave:
    .incbin FIELD_BIN
field_wave_end:
    .balign 4
    .global field_wave_words
field_wave_words:
    .word (field_wave_end - field_wave) / 4
