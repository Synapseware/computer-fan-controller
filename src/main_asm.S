; Interrupt assembly routines


.data

;
;foo:           .zero 1
delay:          .zero 1


.text


; -----------------------------------------------------------------------------
; Event loop monitor, driven by timer2, this ISR is used
; to toggle a global bit once every second.  Timer2 runs
; at 8MHz / 256 / 250 for 125Hz interrupt.  This can
.global TIM2_COMPA_vect

    TIM2_COMPA_vect:
        push    r24
        in      r24, _SFR_IO_ADDR(SREG)
        push    r24

        lds     r24, delay                  ; load the current delay

        cpi     r24, 0                      ; check for 0
        brne    tc2a_dec                    ; branch if not zero

        ; toggle global timer tick variable
        ldi     r24, 1
        sts     _monitorTick, r24

        ldi     r24, TIMER2_DELAY
        sts     delay, r24

    ; Decrements and stores the delay
    tc2a_dec:
        subi    r24, 1
        sts     delay, r24

    tc2a_done:
        pop     r24
        out     _SFR_IO_ADDR(SREG)
        pop     r24

        rei
