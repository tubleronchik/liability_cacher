#!/usr/bin/env python3
import graphene
from graphene import relay, Argument, Connection
from graphene_sqlalchemy import SQLAlchemyObjectType, SQLAlchemyConnectionField
from graphene_sqlalchemy_filter import FilterableConnectionField, FilterSet

from helpers.models import Liability as LiabilityModel


class LiabilityFilter(FilterSet):
    class Meta:
        model = LiabilityModel
        fields = {
                "address":  [...],
                "model": [...],
                "objective": [...],
                "result": [...],
                "promisee": [...],
                "promisor": [...],
                "lighthouse": [...],
                "token": [...],
                "cost": [...],
                "validator": [...],
                "validatorFee": [...],
                }

class MyFilterableConnectionField(FilterableConnectionField):
    filters = {LiabilityModel: LiabilityFilter()}

class LiabilitySchema(SQLAlchemyObjectType):
    class Meta:
        model = LiabilityModel
        interfaces = (relay.Node, )
        connection_field_factory = MyFilterableConnectionField.factory

class LiabilityConnection(Connection):
    class Meta:
        node = LiabilitySchema

class Query(graphene.ObjectType):
    liabilities = MyFilterableConnectionField(LiabilityConnection)

schema = graphene.Schema(query=Query)

