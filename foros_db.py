import sqlite3
import os
import hashlib
import random

DB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "foros.db")

def hash_password(password: str) -> str:
    salt = os.urandom(16)
    key = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt, 100000)
    return salt.hex() + ":" + key.hex()

def verify_password(password: str, hashed: str) -> bool:
    try:
        salt_hex, key_hex = hashed.split(":")
        salt = bytes.fromhex(salt_hex)
        key = bytes.fromhex(key_hex)
        new_key = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt, 100000)
        return new_key == key
    except Exception:
        return False

def init_db():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    
    # 1. Users table
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS users (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        email TEXT UNIQUE NOT NULL,
        password_hash TEXT NOT NULL,
        subscription_status TEXT DEFAULT 'active',
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    )
    """)
    
    # 2. Sessions table
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS sessions (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        user_id INTEGER NOT NULL,
        start_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        end_time TIMESTAMP,
        duration_seconds INTEGER,
        FOREIGN KEY(user_id) REFERENCES users(id)
    )
    """)
    
    # 3. Operations table
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS operations (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        user_id INTEGER,
        robot_brand TEXT NOT NULL,
        op_type TEXT NOT NULL,
        prompt TEXT,
        latency_ms REAL,
        timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        FOREIGN KEY(user_id) REFERENCES users(id)
    )
    """)
    
    # 4. Logs table
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS system_logs (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        level TEXT NOT NULL,
        message TEXT NOT NULL,
        timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    )
    """)
    
    # Check if admin user exists, if not create default
    cursor.execute("SELECT id FROM users WHERE email = 'admin@foros.run'")
    if not cursor.fetchone():
        admin_pwd = hash_password("adminforos2026")
        cursor.execute("INSERT INTO users (email, password_hash) VALUES (?, ?)", ("admin@foros.run", admin_pwd))
        
    # 5. Password Resets table
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS password_resets (
        email TEXT PRIMARY KEY,
        token TEXT NOT NULL,
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    )
    """)

    # 6. Inference log — production data for model retraining
    # Captures every /api/process call: raw model output, corrected JSON,
    # whether the regex post-corrector intervened, and the final generated code.
    # These pairs (prompt => corrected_json) become training examples for v5.3.
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS inference_log (
        id                INTEGER PRIMARY KEY AUTOINCREMENT,
        user_id           INTEGER,
        timestamp         TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        prompt            TEXT NOT NULL,
        format            TEXT NOT NULL DEFAULT 'urscript',
        raw_model_json    TEXT,
        corrected_json    TEXT,
        was_corrected     INTEGER NOT NULL DEFAULT 0,
        coord_fix_count   INTEGER NOT NULL DEFAULT 0,
        final_code        TEXT,
        latency_ms        REAL,
        training_approved INTEGER,
        FOREIGN KEY(user_id) REFERENCES users(id)
    )
    """)

    conn.commit()
    conn.close()

def log_system(level: str, message: str):
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute("INSERT INTO system_logs (level, message) VALUES (?, ?)", (level, message))
        conn.commit()
        conn.close()
    except Exception as e:
        print(f"[db] Failed to log: {e}")

def register_user(email: str, password: str) -> bool:
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        pwd_hash = hash_password(password)
        cursor.execute("INSERT INTO users (email, password_hash) VALUES (?, ?)", (email, pwd_hash))
        conn.commit()
        conn.close()
        log_system("SUCCESS", f"User registered: {email}")
        return True
    except sqlite3.IntegrityError:
        return False
    except Exception as e:
        log_system("ERROR", f"Registration error for {email}: {e}")
        return False

def authenticate_user(email: str, password: str) -> dict:
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute("SELECT id, password_hash, subscription_status FROM users WHERE email = ?", (email,))
        row = cursor.fetchone()
        conn.close()
        
        if row and verify_password(password, row[1]):
            log_system("INFO", f"User logged in: {email}")
            return {"id": row[0], "email": email, "subscription_status": row[2]}
        return None
    except Exception as e:
        log_system("ERROR", f"Auth error for {email}: {e}")
        return None

def open_session(user_id: int) -> int:
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute("INSERT INTO sessions (user_id) VALUES (?)", (user_id,))
        sess_id = cursor.lastrowid
        conn.commit()
        conn.close()
        return sess_id
    except Exception as e:
        log_system("ERROR", f"Failed to open session for user {user_id}: {e}")
        return None

def close_session(session_id: int):
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute("UPDATE sessions SET end_time = CURRENT_TIMESTAMP WHERE id = ?", (session_id,))
        # Calculate duration
        cursor.execute("""
            UPDATE sessions 
            SET duration_seconds = CAST((strftime('%s', end_time) - strftime('%s', start_time)) AS INTEGER)
            WHERE id = ?
        """, (session_id,))
        conn.commit()
        conn.close()
    except Exception as e:
        log_system("ERROR", f"Failed to close session {session_id}: {e}")

def log_operation(user_id: int, robot_brand: str, op_type: str, prompt: str = None, latency_ms: float = 0.0):
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute("""
            INSERT INTO operations (user_id, robot_brand, op_type, prompt, latency_ms)
            VALUES (?, ?, ?, ?, ?)
        """, (user_id, robot_brand, op_type, prompt, latency_ms))
        conn.commit()
        conn.close()
    except Exception as e:
        log_system("ERROR", f"Failed to log operation for user {user_id}: {e}")


def log_inference(
    prompt: str,
    raw_model_json: str,
    corrected_json: str,
    was_corrected: bool,
    coord_fix_count: int,
    final_code: str,
    format: str = 'urscript',
    latency_ms: float = 0.0,
    user_id: int = None
):
    """
    Log a complete inference call for future model retraining.

    Every call to /api/process is recorded here with:
      - prompt            : original operator directive
      - raw_model_json    : JSON string produced by the model (before correction)
      - corrected_json    : JSON string after regex post-processor
      - was_corrected     : True if the corrector changed anything
      - coord_fix_count   : number of individual coordinate values fixed
      - final_code        : URScript/KRL/etc. sent to the robot
      - format            : target robot language
      - latency_ms        : total inference latency

    Rows with was_corrected=False are high-quality training examples.
    Rows with was_corrected=True are valuable for hard-negative training.
    Admin can review and approve rows via the /api/admin/training endpoint.
    """
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute("""
            INSERT INTO inference_log
                (user_id, prompt, format, raw_model_json, corrected_json,
                 was_corrected, coord_fix_count, final_code, latency_ms)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            user_id,
            prompt,
            format,
            raw_model_json,
            corrected_json,
            1 if was_corrected else 0,
            coord_fix_count,
            final_code,
            latency_ms
        ))
        conn.commit()
        conn.close()
    except Exception as e:
        log_system("ERROR", f"Failed to log inference: {e}")


def approve_inference(inference_id: int, approved: bool):
    """Mark an inference log row as approved (1) or rejected (0) for training."""
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute(
            "UPDATE inference_log SET training_approved = ? WHERE id = ?",
            (1 if approved else 0, inference_id)
        )
        conn.commit()
        conn.close()
    except Exception as e:
        log_system("ERROR", f"Failed to approve inference {inference_id}: {e}")


def get_inference_stats() -> dict:
    """Return inference log statistics for the admin dashboard."""
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()

        cursor.execute("SELECT COUNT(*) FROM inference_log")
        total = cursor.fetchone()[0]

        cursor.execute("SELECT COUNT(*) FROM inference_log WHERE was_corrected = 0")
        clean = cursor.fetchone()[0]

        cursor.execute("SELECT COUNT(*) FROM inference_log WHERE was_corrected = 1")
        corrected = cursor.fetchone()[0]

        cursor.execute("SELECT COUNT(*) FROM inference_log WHERE training_approved = 1")
        approved = cursor.fetchone()[0]

        cursor.execute("SELECT AVG(latency_ms) FROM inference_log")
        avg_latency = round(cursor.fetchone()[0] or 0, 1)

        cursor.execute("SELECT AVG(coord_fix_count) FROM inference_log WHERE was_corrected = 1")
        avg_fixes = round(cursor.fetchone()[0] or 0, 2)

        conn.close()
        return {
            "total_inferences": total,
            "clean_model_output": clean,
            "corrector_intervened": corrected,
            "correction_rate_pct": round(corrected / total * 100, 1) if total else 0,
            "approved_for_training": approved,
            "avg_latency_ms": avg_latency,
            "avg_coord_fixes_when_corrected": avg_fixes,
        }
    except Exception as e:
        log_system("ERROR", f"Inference stats query failed: {e}")
        return {}


def export_training_corpus(output_path: str, only_approved: bool = False,
                           include_corrected: bool = True) -> int:
    """
    Export inference log as a JSONL training corpus.

    Each line is in the exact format used by the tokenizer trainer:
        <prompt> => <corrected_json>

    Args:
        output_path     : path to write the .jsonl file
        only_approved   : if True, only export rows with training_approved=1
        include_corrected: if True, include rows where the corrector intervened
                          (uses corrected_json as the gold label)
                          if False, only include rows where model was already correct

    Returns:
        number of lines written
    """
    import json as _json
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()

        conditions = ["corrected_json IS NOT NULL", "prompt IS NOT NULL"]
        if only_approved:
            conditions.append("training_approved = 1")
        if not include_corrected:
            conditions.append("was_corrected = 0")

        where = " AND ".join(conditions)
        cursor.execute(f"""
            SELECT prompt, corrected_json
            FROM inference_log
            WHERE {where}
            ORDER BY id ASC
        """)
        rows = cursor.fetchall()
        conn.close()

        import os as _os
        _os.makedirs(_os.path.dirname(_os.path.abspath(output_path)), exist_ok=True)

        written = 0
        with open(output_path, 'w', encoding='utf-8') as f:
            for prompt, corrected_json in rows:
                # Validate JSON before writing
                try:
                    ops = _json.loads(corrected_json)
                    if not isinstance(ops, list) or not ops:
                        continue
                    # Write in tokenizer training format: prompt => json
                    line = f"{prompt.strip()} => {corrected_json.strip()}"
                    f.write(line + '\n')
                    written += 1
                except Exception:
                    continue  # Skip malformed rows

        log_system("INFO", f"Training corpus exported: {written} examples -> {output_path}")
        return written
    except Exception as e:
        log_system("ERROR", f"Export training corpus failed: {e}")
        return 0


def get_metrics() -> dict:
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        
        # 1. Total users count
        cursor.execute("SELECT COUNT(*) FROM users")
        total_users = cursor.fetchone()[0]
        
        # 2. Active subscriptions
        cursor.execute("SELECT COUNT(*) FROM users WHERE subscription_status = 'active'")
        active_subs = cursor.fetchone()[0]
        
        # 3. Total sessions duration (seconds)
        cursor.execute("SELECT SUM(duration_seconds) FROM sessions WHERE duration_seconds IS NOT NULL")
        total_duration = cursor.fetchone()[0] or 0
        
        # 4. Total operations count
        cursor.execute("SELECT COUNT(*) FROM operations")
        total_ops = cursor.fetchone()[0]
        
        # 5. Brand distribution
        cursor.execute("SELECT robot_brand, COUNT(*) FROM operations GROUP BY robot_brand")
        brands = dict(cursor.fetchall())
        
        # 6. Operations by type
        cursor.execute("SELECT op_type, COUNT(*) FROM operations GROUP BY op_type")
        op_types = dict(cursor.fetchall())
        
        # 7. User list (id, email, subscription_status, created_at, active_session_count)
        cursor.execute("""
            SELECT u.id, u.email, u.subscription_status, u.created_at, COUNT(s.id)
            FROM users u
            LEFT JOIN sessions s ON u.id = s.user_id
            GROUP BY u.id
        """)
        users_list = []
        for r in cursor.fetchall():
            users_list.append({
                "id": r[0],
                "email": r[1],
                "subscription_status": r[2],
                "created_at": r[3],
                "session_count": r[4]
            })
            
        conn.close()
        return {
            "total_users": total_users,
            "active_subscriptions": active_subs,
            "total_duration_hours": round(total_duration / 3600.0, 2),
            "total_operations": total_ops,
            "brands": brands,
            "op_types": op_types,
            "users": users_list
        }
    except Exception as e:
        log_system("ERROR", f"Metrics query failed: {e}")
        return {}

def get_logs(limit: int = 100) -> list:
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute("SELECT level, message, timestamp FROM system_logs ORDER BY id DESC LIMIT ?", (limit,))
        logs = [{"level": r[0], "message": r[1], "timestamp": r[2]} for r in cursor.fetchall()]
        conn.close()
        return logs
    except Exception as e:
        print(f"[db] Failed to fetch logs: {e}")
        return []

def toggle_user_subscription(user_id: int) -> bool:
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute("SELECT subscription_status FROM users WHERE id = ?", (user_id,))
        row = cursor.fetchone()
        if row:
            new_status = 'inactive' if row[0] == 'active' else 'active'
            cursor.execute("UPDATE users SET subscription_status = ? WHERE id = ?", (new_status, user_id))
            conn.commit()
            conn.close()
            log_system("INFO", f"Subscription status for user ID {user_id} toggled to {new_status}")
            return True
        conn.close()
        return False
    except Exception as e:
        log_system("ERROR", f"Failed to toggle subscription for user {user_id}: {e}")
        return False

def generate_reset_token(email: str) -> str:
    try:
        # Check if user exists
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute("SELECT id FROM users WHERE email = ?", (email,))
        user = cursor.fetchone()
        if not user:
            conn.close()
            return None
        
        # Generate 6-digit code
        code = f"{random.randint(100000, 999999)}"
        
        # Insert or replace
        cursor.execute("INSERT OR REPLACE INTO password_resets (email, token) VALUES (?, ?)", (email, code))
        conn.commit()
        conn.close()
        
        log_system("WARNING", f"Password reset requested for {email}. Recovery code: {code}")
        return code
    except Exception as e:
        log_system("ERROR", f"Failed to generate reset token for {email}: {e}")
        return None

def verify_and_reset_password(email: str, token: str, new_password: str) -> bool:
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        
        # Check if reset code exists
        cursor.execute("SELECT token FROM password_resets WHERE email = ?", (email,))
        row = cursor.fetchone()
        if not row:
            conn.close()
            return False
            
        stored_token = row[0]
        if stored_token != token:
            conn.close()
            return False
            
        # Update user's password
        pwd_hash = hash_password(new_password)
        cursor.execute("UPDATE users SET password_hash = ? WHERE email = ?", (pwd_hash, email))
        
        # Delete reset entry
        cursor.execute("DELETE FROM password_resets WHERE email = ?", (email,))
        conn.commit()
        conn.close()
        
        log_system("SUCCESS", f"Password reset successfully for user: {email}")
        return True
    except Exception as e:
        log_system("ERROR", f"Reset password error for {email}: {e}")
        return False
