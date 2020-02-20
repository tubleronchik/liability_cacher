#!/usr/bin/env python3
import graphene
from graphene_sqlalchemy import SQLAlchemyObjectType

from db_helper.liability import Liability as L

class LiabilitySchema(SQLAlchemyObjectType):
    class Meta:
        model = L

class Query(graphene.ObjectType):
    liabilities = graphene.List(LiabilitySchema)

    def resolve_liabilities(self, info):
        query = LiabilitySchema.get_query(info)  # SQLAlchemy query
        return query.all()

schema = graphene.Schema(query=Query)

