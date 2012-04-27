all: xbow440

.PHONY: xbow440
ifndef ROS_ROOT
xbow440: no_ros
else
xbow440:
include $(shell rospack find mk)/cmake.mk
endif

.PHONY: no_ros
no_ros:
	@mkdir -p build
	-mkdir -p bin
	cd build && cmake -DBUILD_WITH_ROS=OFF ..
ifneq ($(MAKE),)
	cd build && $(MAKE)
else
	cd build && make
endif
