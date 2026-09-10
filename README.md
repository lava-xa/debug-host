### 快速开始

- 这是aerohand灵巧手内部电机控制的上位机程序，主要供debug用
- 主要基于原项目python sdk开发，使用uv管理依赖，python版本>=3.11

### 使用指南

确保你安装了uv和python

1、克隆此项目
```
git clone https://github.com/lava-xa/debug-host.git
```

2、创建虚拟环境并安装依赖
```
uv venv
uv pip install -r requirements.txt
```
3、运行gui调试
```
uv run python3 main.py
```

