# ================================================================================
#  Makefile für das Programmieraufgaben "Echtzeitprogrammierung" - Philipp Nöcker
# ================================================================================

# Compiler und Flags
CC      := gcc
CFLAGS  := -O2 -Wall -Wextra -pthread
LDFLAGS :=

# Alle .c-Dateien im Projekt finden
SRC := $(shell find . -type f -name '*.c')

# Zielordner für Builds
BUILDDIR := build

# Für jede .c-Datei wird ein gleichnamiges Binary im build/-Ordner erstellt
TARGETS := $(addprefix $(BUILDDIR)/, $(patsubst %.c, %, $(SRC)))

# Standardziel: alles bauen
all: $(TARGETS)

# Regel: .c -> Binary im selben Unterordner
$(BUILDDIR)/%: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

# Hilfsziel: bestimmtes Programm starten
run:
	@if [ -z "$(NAME)" ]; then echo "Bitte NAME=<Pfad/zur/Datei> angeben (z.B. NAME=Sockets/peer)"; exit 1; fi
	@exec $(BUILDDIR)/$(NAME)

# Aufräumen
clean:
	rm -rf $(BUILDDIR)

.PHONY: all clean run
