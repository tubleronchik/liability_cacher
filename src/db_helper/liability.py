from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, Integer, String, TIMESTAMP, Boolean
from sqlalchemy.sql import func


Base = declarative_base()

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
       return "<Liability (address='%s', model='%s', lighthouse='%s', result='%s')>" %
                        (self.address, self.model, self.lighthouse, self.result)

