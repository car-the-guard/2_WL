# 프로그램 이름
TARGET = v2x_vT11

# 컴파일러 및 옵션
CC = gcc
CFLAGS = -Wall -Wextra -pthread -I./include
LDFLAGS = -pthread -lm

# [중요] 시뮬레이션 모드 활성화 여부 (1이면 활성, 0이면 하드웨어 모드)
# 실행 시 'make SIM=0' 으로 하드웨어 모드 컴파일 가능
SIM = 0
ifeq ($(SIM), 1)
    CFLAGS += -DSIMULATION
endif

# 폴더 설정
SRCDIR = ./src
INCDIR = ./include
OBJDIR = ./obj

# 소스 파일 및 오브젝트 파일 목록
SRCS = $(wildcard $(SRCDIR)/*.c)
OBJS = $(SRCS:$(SRCDIR)/%.c=$(OBJDIR)/%.o)

# 기본 실행 타겟
all: $(OBJDIR) $(TARGET)

# 타겟 생성
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "------------------------------------------"
	@echo " Build Complete: $(TARGET) (SIMULATION=$(SIM))"
	@echo "------------------------------------------"

# 각 소스 파일을 오브젝트 파일로 컴파일
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# 오브젝트 폴더 생성
$(OBJDIR):
	mkdir -p $(OBJDIR)

# 정리
clean:
	rm -rf $(OBJDIR) $(TARGET)
	@echo " Clean Complete."

.PHONY: all clean