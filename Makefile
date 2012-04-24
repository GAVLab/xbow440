all: xbow440

install:
	cd build && make install

uninstall:
	cd build && make uninstall

xbow440:
	@mkdir -p build
	-mkdir -p bin
	cd build && cmake $(CMAKE_FLAGS) ..
ifneq ($(MAKE),)
	cd build && $(MAKE)
else
	cd build && make
endif

clean:
	-cd build && make clean
	rm -rf build bin lib

.PHONY: test
test:
	@mkdir -p build
	@mkdir -p bin
	cd build && cmake $(CMAKE_FLAGS) -DXBOW_BUILD_TESTS=1 -DXBOW_BUILD_EXAMPLES=1 ..
ifneq ($(MAKE),)
	cd build && $(MAKE)
else
	cd build && make
endif

