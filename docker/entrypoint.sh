#!/usr/bin/env bash
set -e

supervisord -c /etc/supervisor/supervisord.conf

exec "$@"
