#!/usr/bin/env python3
import graphene
from graphene import relay
from graphene_sqlalchemy import SQLAlchemyObjectType, SQLAlchemyConnectionField

from db_helper.liability import Liability as LiabilityModel

class LiabilitySchema(SQLAlchemyObjectType):
    class Meta:
        model = LiabilityModel
        # interfaces = (relay.Node, )

class Query(graphene.ObjectType):
    # node = relay.Node.Field()
    # liabilities = SQLAlchemyConnectionField(LiabilitySchema)

    liabilities = graphene.List(LiabilitySchema, model=graphene.String(default_value=None, required=False))

    def resolve_liabilities(self, info, **kwargs):
        model = kwargs.get('model', None)
        print(model)
        q = LiabilitySchema.get_query(info)
        print(dir(q))
        if model is None:
            return q.all()

        print(q.filter(LiabilityModel.model == model))

        return q.filter(LiabilityModel.model == model).limit(1)

schema = graphene.Schema(query=Query)

