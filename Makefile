.DEFAULT_GOAL := all
PLUGIN_DIR := bearamin-virtual/vcv/BearaminVirtual

.PHONY: all dep clean cleandep dist
all dep cleandep:
	$(MAKE) -C $(PLUGIN_DIR) $@

clean:
	$(MAKE) -C $(PLUGIN_DIR) clean
	rm -rf dist

dist:
	$(MAKE) -C $(PLUGIN_DIR) dist
	mkdir -p dist
	cp $(PLUGIN_DIR)/dist/*.vcvplugin dist/
