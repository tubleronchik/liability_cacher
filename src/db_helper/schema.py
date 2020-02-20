#!/usr/bin/env python3
import graphene
from graphene import relay
from graphene_sqlalchemy import SQLAlchemyObjectType, SQLAlchemyConnectionField

from db_helper.liability import Liability as LiabilityModel

class LiabilitySchema(SQLAlchemyObjectType):
    class Meta:
        model = LiabilityModel
        interfaces = (relay.Node, )

class Query(graphene.ObjectType):
    node = relay.Node.Field()
    liabilities = SQLAlchemyConnectionField(LiabilitySchema)

schema = graphene.Schema(query=Query)

