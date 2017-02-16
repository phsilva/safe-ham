
// TODO: add printf support (USART/USB)
// TODO: add FreeRTOS (task manaager, USB, DW1000, others)

// TODO: clear interruptions while sending SPI to DW1000
//  CM_ATOMIC_BLOCK in cortex.h

// understand interrupts
// 	nvic_set_priority(NVIC_DMA1_CHANNEL7_IRQ, 0);
//  nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ);
//  convention defines the interrupt service routines name (irq_name__isr)
//  per chip defined in vector_nvic.h

// understand USB CDC
// undestand systick
// understand data format DW1000
// how to delays and execute?
