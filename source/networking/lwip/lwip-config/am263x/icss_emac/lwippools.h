/* OPTIONAL: Pools to replace heap allocation
 * Optional: Pools can be used instead of the heap for mem_malloc. If
 * so, these should be defined here, in increasing order according to
 * the pool element size.
 *
 * LWIP_MALLOC_MEMPOOL(number_elements, element_size)
 *
 * Note: pbuf_malloc() applies memory alignment (LWIP_MEM_ALIGN_SIZE()) separately
 * to pbuf struct, layer header and payload.  For example, for MEM_ALIGMENT of 128B,
 * this means that a payload of 1460B actually needs a buffer 1792B as follows:
 *  - pbuf struct: size is 16B, due to alignment required an allocation of 128B
 *  - layer hader: different sizes but < 128B, this also becomes 128B due to aligment
 *  - payload: i.e. 1460B becomes 1536B after 128B aligment
 *
 * So total buffer size is now: 1536 + 128 + 128 = 1792B.
 */

#if MEM_USE_POOLS
LWIP_MALLOC_MEMPOOL_START
LWIP_MALLOC_MEMPOOL(100, 128)
LWIP_MALLOC_MEMPOOL(18, 1792)
LWIP_MALLOC_MEMPOOL(4, 4096)
LWIP_MALLOC_MEMPOOL_END
#endif /* MEM_USE_POOLS */

/* Optional: Your custom pools can go here if you would like to use
 * lwIP's memory pools for anything else.
 */
LWIP_MEMPOOL(SYS_MBOX, 22, 100, "SYS_MBOX")
