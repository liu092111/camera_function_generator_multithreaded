# -*- coding: utf-8 -*-
"""
執行緒安全的共享狀態模組
提供多執行緒間安全的狀態共享機制
"""

import threading


class ThreadSafeState:
    """
    執行緒安全的共享狀態類別
    
    使用方式與字典相同，支援 [] 存取：
        state = ThreadSafeState()
        state['recording'] = True
        if state['recording']:
            ...
    """
    
    def __init__(self, initial_state=None):
        """
        初始化執行緒安全狀態
        
        Args:
            initial_state: 初始狀態字典（可選）
        """
        self._lock = threading.Lock()
        self._state = initial_state.copy() if initial_state else {}
    
    def get(self, key, default=None):
        """
        執行緒安全的讀取
        
        Args:
            key: 鍵值
            default: 預設值（如果 key 不存在）
        
        Returns:
            對應的值或預設值
        """
        with self._lock:
            return self._state.get(key, default)
    
    def set(self, key, value):
        """
        執行緒安全的寫入
        
        Args:
            key: 鍵值
            value: 要設定的值
        """
        with self._lock:
            self._state[key] = value
    
    def update(self, **kwargs):
        """
        執行緒安全的批次更新
        
        Args:
            **kwargs: 要更新的鍵值對
        """
        with self._lock:
            self._state.update(kwargs)
    
    def __getitem__(self, key):
        """支援 state['key'] 語法"""
        return self.get(key)
    
    def __setitem__(self, key, value):
        """支援 state['key'] = value 語法"""
        self.set(key, value)
    
    def __contains__(self, key):
        """支援 'key' in state 語法"""
        with self._lock:
            return key in self._state
    
    def keys(self):
        """返回所有鍵值（執行緒安全）"""
        with self._lock:
            return list(self._state.keys())
    
    def values(self):
        """返回所有值（執行緒安全）"""
        with self._lock:
            return list(self._state.values())
    
    def items(self):
        """返回所有項目（執行緒安全）"""
        with self._lock:
            return list(self._state.items())
    
    def copy(self):
        """返回狀態的副本（執行緒安全）"""
        with self._lock:
            return self._state.copy()
    
    def __repr__(self):
        with self._lock:
            return f"ThreadSafeState({self._state})"
