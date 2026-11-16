# Official controller environment: what every submission runs in on the scoring server.
#   docker build -f docker/Dockerfile -t aoc-drone-racing:2026 .
# The simulator itself runs on the host (same pinned versions); only the student
# controller runs in this image, with no network, a read-only file system, one CPU
# and a memory cap (see quad_race/evaluate.py: docker_prefix).
FROM python:3.12.12-slim

ENV PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1 PYTHONDONTWRITEBYTECODE=1
WORKDIR /opt/quad-race
COPY requirements.txt ./
RUN pip install -r requirements.txt
COPY pyproject.toml ./
COPY src ./src
RUN pip install --no-deps . && useradd --create-home racer
USER racer
WORKDIR /submission
