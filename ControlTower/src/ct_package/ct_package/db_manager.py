import mysql.connector
from mysql.connector import Error

class DBManager:
    def __init__(self, host, user, password, database):
        self.host = host
        self.user = user
        self.password = password
        self.database = database
        self.conn = None
        
    def create_connection(self):
        try:
            self.conn = mysql.connector.connect(
                host=self.host,
                user=self.user,
                password=self.password,
                database=self.database
            )
            if self.conn.is_connected():
                print(f"Connected to MySQL database {self.database}")
        except Error as e:
            print(f"Error connecting to database: {e}")
        return self.conn

    def close_connection(self):
        """Close the database connection."""
        if self.conn.is_connected():
            self.conn.close()
            print("MySQL connection is closed")

    def execute_query(self, query, params=None):
        conn = self.create_connection()
        try:
            cursor = conn.cursor()
            if params:
                cursor.execute(query, params)
            else:
                cursor.execute(query)
            conn.commit()
            print("Query executed successfully")
        except Error as e:
            print(f"Error executing query: {e}")
        finally:
            cursor.close()
            conn.close()

    def fetch_query(self, query, params=None):
        conn = self.create_connection()
        try:
            cursor = conn.cursor()
            if params:
                cursor.execute(query, params)
            else:
                cursor.execute(query)
            result = cursor.fetchall()
            return result
        except Error as e:
            print(f"Error fetching query: {e}")
            return None
        finally:
            cursor.close()
            conn.close()