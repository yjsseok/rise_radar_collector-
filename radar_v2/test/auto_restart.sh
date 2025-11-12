#!/bin/bash
#
# SENSR LiDAR Recorder - 자동 재시작 스크립트
# 30분마다 프로그램을 재시작하여 메모리 누수 방지
#

# 설정
DURATION=1800        # 30분 (초)
HOST="samyang"       # SENSR 호스트 ID
RESTART_DELAY=5      # 재시작 대기 시간 (초)

# 색상 코드
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 세션 카운터
SESSION=0

echo "======================================================================="
echo "🚀 SENSR LiDAR Recorder - 자동 재시작 모드"
echo "======================================================================="
echo "호스트: ${HOST}"
echo "세션 시간: ${DURATION}초 ($(($DURATION / 60))분)"
echo "재시작 대기: ${RESTART_DELAY}초"
echo "======================================================================="
echo ""

# Ctrl+C 처리
trap 'echo -e "\n${RED}⚠️  사용자 중단. 프로그램 종료...${NC}"; exit 0' INT TERM

while true; do
    SESSION=$((SESSION + 1))

    echo -e "${GREEN}=======================================================================${NC}"
    echo -e "${GREEN}🚀 세션 ${SESSION} 시작: $(date '+%Y-%m-%d %H:%M:%S')${NC}"
    echo -e "${GREEN}=======================================================================${NC}"
    echo ""

    # 메인 프로그램 실행 (timeout으로 30분 후 자동 종료)
    timeout ${DURATION} python main_optimized.py --host ${HOST} --duration ${DURATION}

    EXIT_CODE=$?

    echo ""
    echo -e "${YELLOW}=======================================================================${NC}"
    echo -e "${YELLOW}🔄 세션 ${SESSION} 종료: $(date '+%Y-%m-%d %H:%M:%S')${NC}"
    echo -e "${YELLOW}   종료 코드: ${EXIT_CODE}${NC}"

    # 종료 코드 해석
    if [ $EXIT_CODE -eq 124 ]; then
        echo -e "${YELLOW}   이유: 정상 타임아웃 (${DURATION}초)${NC}"
    elif [ $EXIT_CODE -eq 0 ]; then
        echo -e "${GREEN}   이유: 정상 종료${NC}"
    else
        echo -e "${RED}   이유: 오류 종료 (코드: ${EXIT_CODE})${NC}"
    fi

    echo -e "${YELLOW}   ${RESTART_DELAY}초 후 재시작...${NC}"
    echo -e "${YELLOW}=======================================================================${NC}"
    echo ""

    # 재시작 전 대기
    sleep ${RESTART_DELAY}
done
