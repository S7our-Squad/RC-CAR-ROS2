## 🪟 Windows Setup & Run Instructions

### 1️⃣ Create and activate virtual environment
```bash
python -m venv whisper
whisper\Scripts\activate
pip install -r requirements.txt
```

### terminal 1- input node
```bash
python run input.py
```

### Terminal 2 – Controller Module

> **NOTE:**  
> Update the **IP address** in `controller.py` before running.  
> Default port: **5000**

```bash
python run controller.py
