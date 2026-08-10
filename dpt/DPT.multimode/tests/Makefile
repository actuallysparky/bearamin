TEST_BUILD_DIR = ../../../_local/test-build/DPT.multimode
TEST_BIN = $(TEST_BUILD_DIR)/multimode_engine_test

.PHONY: test clean

test: $(TEST_BIN)
	$(TEST_BIN)

$(TEST_BIN): multimode_engine_test.cpp ../MultimodeEngine.h ../BearaminPredictor.h
	mkdir -p $(TEST_BUILD_DIR)
	c++ -std=c++14 -Wall -Wextra -Werror -I.. multimode_engine_test.cpp -o $(TEST_BIN)

clean:
	rm -rf $(TEST_BUILD_DIR)
