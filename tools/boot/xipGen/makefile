
ifeq ($(OS),Windows_NT)
  EXE_FILE = xipGen.exe 
  RM=del
else
  UNAME_S = $(shell uname -s)
  ifeq ($(UNAME_S), Darwin)
	EXE_FILE = xipGen.out.mac
else
	EXE_FILE = xipGen.out
endif
  RM=rm -f
endif

%.exe %.out %.out.mac: %.c
	gcc $< -o $@

all: $(EXE_FILE)

clean:
	$(RM) $(EXE_FILE)
	
	
