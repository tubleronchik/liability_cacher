#!/usr/bin/env python3
import graphene
from graphene import relay
from graphene_sqlalchemy import SQLAlchemyObjectType, SQLAlchemyConnectionField

from helpers.models import Liability as LiabilityModel

class LiabilitySchema(SQLAlchemyObjectType):
    class Meta:
        model = LiabilityModel
        # interfaces = (relay.Node, )

class Query(graphene.ObjectType):
    # node = relay.Node.Field()
    # liabilities = SQLAlchemyConnectionField(LiabilitySchema)

    liabilities = graphene.List(LiabilitySchema,
                    model=graphene.String(required=False),
                    limit=graphene.Int(required=False))

    def resolve_liabilities(self, info, **kwargs):
        model = kwargs.get('model', None)
        limit = kwargs.get("limit", None)

        q = LiabilitySchema.get_query(info)

        if model is not None:
            q = q.filter(LiabilityModel.model == model)

        if limit is not None:
            q = q.limit(limit)
        else:
            q = q.all()

        return q

schema = graphene.Schema(query=Query)

