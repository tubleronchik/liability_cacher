#!/usr/bin/env python3
import sqlite3
from sqlite3.dbapi2 import connect
import contextlib
import rospy


class DataBase():
    
    def connection(self):
        db_file = rospy.get_param("/liability_cacher/datalog_cacher/db_path")
        try:
            connection = sqlite3.connect(db_file)
        
        except sqlite3.Error as e:
            print(f"Could not connect to data base {e}")

        return connection

    def create_table(self):
        connection = self.connection()
        cursor = connection.cursor()
        cursor.execute(
                        """
                        CREATE TABLE IF NOT EXISTS datalog (
                                                    id integer PRIMARY KEY,
                                                    status text,
                                                    hash text,
                                                    address text,
                                                    promisee text
                                                    time real
                            ); """     
        )

    def add_data(self, status, hash, address, promisee, time):
        connection = self.connection()
        cursor = connection.cursor()
        with contextlib.closing(connection) as conn: # auto-closes
            with conn: # auto-commits
                with contextlib.closing(cursor) as cursor: # auto-closes
                    cursor.execute("INSERT INTO datalog (status, hash, address, promisee, time) VALUES (?, ?, ?, ?, ?)", (status, hash, address, promisee, time))


    def update_status(self, status, hash):
        connection = self.connection()
        cursor = connection.cursor()
        with contextlib.closing(connection) as conn: # auto-closes
            with conn: # auto-commits
                with contextlib.closing(cursor) as cursor: # auto-closes
                    cursor.execute("UPDATE datalog SET status = ? WHERE hash = ?", (status, hash))



    def checker(self, current_time):
        connection = self.connection()
        cursor = connection.cursor()
        time = current_time - 7200 # 2hrs
        with contextlib.closing(connection) as conn: # auto-closes
            with conn: # auto-commits
                with contextlib.closing(cursor) as cursor: # auto-closes
                    hashes = cursor.execute("SELECT hash, address, promisee FROM datalog WHERE time < ? AND status='not upload'", (time,)).fetchall()
        return hashes

