# Liability Cacher

It listens to all new liability events and stores them into a database.
Provides (GraphQL)[https://graphql.org/] API for data requests.

## GraphQL Schema

```
{
  liabilities(model: "QmaTLSFvFh2gTv5eUDfpE1YZjiwuPwSq9RpSG9kSJ6Y9W9", limit: 1) {
    id
    model
    lighthouse
  }
}
```

Liabilities' arguments:

* `model` - optional, model hash
* `limit` - optional, how many results. If not provided, returns all records
