#!/usr/bin/python3
from LoggerFabric.LoggerFabric import LoggerFabric

logger_fabric = LoggerFabric(
    basic_level="info",
    logs_dir = "../../logs",
)

logger = logger_fabric.get_logger("test", use_stderr=True)

logger.info("Hello")
logger.warning("Hello")
logger.error("Hello")
logger.debug("Hello")