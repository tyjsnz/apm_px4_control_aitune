# -*- coding: utf-8 -*-
"""python -m apm_control_api [--host 0.0.0.0 --port 8000]"""
import argparse

import uvicorn

from . import config


def main():
    parser = argparse.ArgumentParser(description='APM 多地面站控制 API')
    parser.add_argument('--host', default=config.HOST)
    parser.add_argument('--port', type=int, default=config.PORT)
    parser.add_argument('--reload', action='store_true')
    args = parser.parse_args()
    uvicorn.run('apm_control_api.app:app', host=args.host, port=args.port,
                reload=args.reload)


if __name__ == '__main__':
    main()
