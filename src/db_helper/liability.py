import rospy
import json

from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, Integer, String, TIMESTAMP, Boolean
from sqlalchemy.sql import func
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, scoped_session


Base = declarative_base()
engine = create_engine(rospy.get_param("/liability_cacher/cacher/db_credentials")[:-1], echo=True)
db_session = scoped_session(sessionmaker(autocommit=False,
                                    autoflush=False,
                                    bind=engine))
Base.query = db_session.query_property()


class Liability(Base):
    __tablename__ = "liabilities"

    id = Column(Integer, primary_key=True)

    address = Column(String)
    model = Column(String)
    objective = Column(String)
    result = Column(String)
    promisee = Column(String)
    promisor = Column(String)
    lighthouse = Column(String)
    # lighthouseFee = Column(String)
    token = Column(String)
    cost = Column(String)
    validator = Column(String)
    validatorFee = Column(String)
    # isSuccess = Column(Boolean)
    # isFinalized = Column(Boolean)

    timestamp = Column(TIMESTAMP(True), server_default=func.now())

    def __get__(self, instance, owner):
        return {
                self.address,
                self.model,
                self.objective,
                self.result,
                self.promisee,
                self.promisor,
                self.lighthouse,
                self.token,
                self.token,
                self.cost,
                self.validator,
                self.validatorFee
                }

    def __repr__(self):
        return json.dumps({
            "id": self.id,
            "address": self.address,
            "model": self.model,
            "objective": self.objective,
            "result": self.result,
            "promisee": self.promisee,
            "promisor": self.promisor,
            "lighthouse": self.lighthouse,
            "token": self.token,
            "cost": self.cost,
            "validator": self.validator,
            "validatorFee": self.validatorFee
            })

