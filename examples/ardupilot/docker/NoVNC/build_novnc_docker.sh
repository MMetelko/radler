#!/bin/bash

cd "$(dirname "$0")" || exit 1

cd Context || exit 1


docker buildx build --no-cache -t novnc:rite .
