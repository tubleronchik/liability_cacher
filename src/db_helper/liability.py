import rospy

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


    def __repr__(self):
       return "<Liability (address='%s', model='%s', lighthouse='%s', result='%s')>" % \
                        (self.address, self.model, self.lighthouse, self.result)

