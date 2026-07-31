mkdir -p ./data

# create fresh database
export DATABASE_URL="sqlite://./data/schema_gen_db.sqlite?mode=rwc"
sea-orm-cli migrate --migration-dir ./packages/backend/migration fresh
# generate entity crate
sea-orm-cli generate entity \
    --output-dir ./packages/backend/entity/src \
    --lib \
    --entity-format dense

# remove schema gen db
rm ./data/schema_gen_db.sqlite