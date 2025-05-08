#!/usr/bin/env python3
"""
数据库管理工具
===========

管理导航点数据库的创建、查询和修改。
"""

import sqlite3
import os
import threading
from datetime import datetime

class DatabaseManager:
    """管理导航点数据库"""
    
    def __init__(self, db_path):
        """初始化数据库管理器
        
        参数:
            db_path (str): 数据库文件路径
        """
        self.db_path = db_path
        self.db_lock = threading.Lock()
        
        # 确保目录存在
        os.makedirs(os.path.dirname(db_path), exist_ok=True)
        
        # 创建表（如果不存在）
        self.create_tables()
    
    # ... [类的剩余方法] ...