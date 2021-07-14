import rospy
import json

from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, Integer, String, TIMESTAMP, Boolean, ForeignKey, Sequence
from sqlalchemy.sql import func
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, scoped_session, relationship
from sqlalchemy.pool import NullPool

#seq = Sequence('id',start=190)
Base = declarative_base()
engine = create_engine(rospy.get_param("/liability_cacher/cacher/db_credentials")[:-1], poolclass=NullPool)
db_session = scoped_session(sessionmaker(autocommit=True,
                                    autoflush=False,
                                    bind=engine))
Base.query = db_session.query_property()


class Liability(Base):
    __tablename__ = "liabilities"

#    seq = Sequence('id', start=190)
    id = Column(Integer, autoincrement=True, primary_key=True)

    address         = Column(String, unique=True)
    model           = Column(String)
    model_data      = Column(String)
    objective       = Column(String)
    objective_data  = Column(String)
    result          = Column(String)
    result_data     = Column(String)
    promisee        = Column(String)
    promisor        = Column(String)
    lighthouse      = Column(String)
    # lighthouseFee = Column(String)
    token           = Column(String)
    cost            = Column(String)
    validator       = Column(String)
    validatorFee    = Column(String)
    # isSuccess = Column(Boolean)
    # isFinalized = Column(Boolean)

    timestamp = Column(TIMESTAMP(True), server_default=func.now())

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

