mkdir -p ./data

# create fresh database
export DATABASE_URL="sqlite://./data/schema_gen_db.sqlite?mode=rwc"
sea-orm-cli migrate fresh
# generate entity crate
sea-orm-cli generate entity \
    --output-dir ./entity/src \
    --lib \
    --entity-format dense

# remove schema gen db
rm ./data/schema_gen_db.sqlite