px4_add_module(
    MODULE examples__lqr_att_controller
    MAIN lqr_att_controller
    STACK_MAX 3500
	COMPILE_FLAGS
    SRCS
        lqr_att_controller.cpp
    DEPENDS
   		px4_work_queue	
    )

