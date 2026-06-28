use sea_orm_migration::{prelude::*, schema::*};

pub struct Migration;

impl MigrationName for Migration {
    fn name(&self) -> &str {
        "m20260628_205535_create_initial_tables"
    }
}

#[async_trait::async_trait]
impl MigrationTrait for Migration {
    async fn up(&self, _manager: &SchemaManager) -> Result<(), DbErr> {

        _manager
            .create_table(
                Table::create()
                    .table("part")
                    .if_not_exists()
                    .col(pk_auto("id"))
                    .col(ColumnDef::new("name").string().null())
                    .col(ColumnDef::new("label_text").string().null())
                    .to_owned(),
            )
            .await?;

        _manager
            .create_table(
                Table::create()
                    .table("item")
                    .if_not_exists()
                    .col(pk_auto("id"))
                    .col(ColumnDef::new("part_id").integer().not_null())
                    .col(ColumnDef::new("quantity").float().not_null())
                    .foreign_key(ForeignKey::create()
                        .name("part_fk")
                        .from("item", "part_id")
                        .to("part", "id")
                        .on_delete(ForeignKeyAction::Cascade)
                    )
                    .to_owned(),
            )
            .await
    }

    async fn down(&self, _manager: &SchemaManager) -> Result<(), DbErr> {
        _manager
            .drop_table(Table::drop().table("part").to_owned())
            .await?;

        _manager
            .drop_table(Table::drop().table("item").to_owned())
            .await
    }
}
