			lui t1, %hi(0x3000)				# First data memory address
			addi t1, t1, %lo(0x3000)
		
			lui t0, %hi(0xAABBCCDD)			# Value to store
			addi t0, t0, %lo(0xAABBCCDD)
			
			sw t0, 0(t1)					# Store value
			lw t2, 0(t1)					# Load value
			
.loop:	j .loop
	