use sea_orm_migration::{prelude::*, schema::*};

pub struct Migration;

impl MigrationName for Migration {
    // 7657f50 and c93a745
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
                    .col(ColumnDef::new("parent_part_id").integer())
                    .col(ColumnDef::new("category_id").integer())
                    .col(ColumnDef::new("name").string())
                    .col(ColumnDef::new("description").string())
                    .col(ColumnDef::new("label_text").string())
                    .col(ColumnDef::new("concrete").boolean().default(true).not_null())
                    .col(ColumnDef::new("image").string())
                    .foreign_key(ForeignKey::create()
                        .name("fk_part_parent_part_id")
                        .from("part", "parent_part_id")
                        .to("part", "id")
                        .on_delete(ForeignKeyAction::Cascade)
                    )
                    .foreign_key(ForeignKey::create()
                        .name("fk_part_category_id")
                        .from("part", "category_id")
                        .to("category", "id")
                        .on_delete(ForeignKeyAction::Restrict)  // reassign to parent before or allow user to choose
                    )
                    .to_owned(),
            )
            .await?;

        _manager
            .create_table(
                Table::create()
                    .table("item")
                    .if_not_exists()
                    .col(pk_auto("id"))
                    .col(ColumnDef::new("quantity").double().not_null())
                    .col(ColumnDef::new("part_id").integer().not_null())
                    .col(ColumnDef::new("location_id").integer().not_null())
                    .col(ColumnDef::new("owner_id").integer())
                    .foreign_key(ForeignKey::create()
                        .name("fk_item_part_id")
                        .from("item", "part_id")
                        .to("part", "id")
                        .on_delete(ForeignKeyAction::Cascade)
                    )
                    .foreign_key(ForeignKey::create()
                        .name("fk_item_location_id")
                        .from("item", "location_id")
                        .to("location", "id")
                        .on_delete(ForeignKeyAction::Restrict)  // in application: reassign to parent (or allow user to choose)
                    )
                    .foreign_key(ForeignKey::create()
                        .name("fk_item_owner_id")
                        .from("item", "owner_id")
                        .to("legal_entity", "id")
                        .on_delete(ForeignKeyAction::SetNull)
                    )
                    .to_owned(),
            )
            .await?;

        _manager
            .create_table(
                Table::create()
                    .table("location")
                    .if_not_exists()
                    .col(pk_auto("id"))
                    .col(ColumnDef::new("parent_location_id").integer())
                    .col(ColumnDef::new("name").string().not_null())
                    .col(ColumnDef::new("description").string())
                    .col(ColumnDef::new("label_text").string())
                    .foreign_key(ForeignKey::create()
                        .name("fk_location_parent_location_id")
                        .from("location", "parent_location_id")
                        .to("location", "id")
                        .on_delete(ForeignKeyAction::Restrict)  // reassign to parent or allow user to choose
                    )
                    .to_owned(),
            )
            .await?;

         _manager
            .create_table(
                Table::create()
                    .table("category")
                    .if_not_exists()
                    .col(pk_auto("id"))
                    .col(ColumnDef::new("parent_category_id").integer())
	                .col(ColumnDef::new("name").string().not_null())
	                .col(ColumnDef::new("description").string())
	                .col(ColumnDef::new("label_text").string())
                    .foreign_key(ForeignKey::create()
                        .name("fk_category_parent_category_id")
                        .from("category", "parent_category_id")
                        .to("category", "id")
                        .on_delete(ForeignKeyAction::Restrict)  // reassign to parent or let user choose
                    )
                    .to_owned(),
            )
            .await?;

        _manager
            .create_table(
                Table::create()
                    .table("parameter")
                    .if_not_exists()
                    .col(pk_auto("id"))
                    .col(ColumnDef::new("name").string().not_null())
                    .col(ColumnDef::new("type").integer().not_null())   // TODO enum somehow?
                    .col(ColumnDef::new("symbol").string())
                    .col(ColumnDef::new("unit").string())
                    .to_owned(),
            )
            .await?;

        _manager
            .create_table(
                Table::create()
                    .table("part_parameter_definition")
                    .if_not_exists()
                    .col(ColumnDef::new("parameter_id").integer().not_null())
                    .col(ColumnDef::new("part_id").integer().not_null())
                    .col(ColumnDef::new("default_value_real").double())
                    .col(ColumnDef::new("default_value_string").string())
                    .col(ColumnDef::new("default_value_int").integer())
                    .primary_key(Index::create()
                        .col("parameter_id")
                        .col("part_id")
                    )
                    .foreign_key(ForeignKey::create()
                        .name("fk_part_parameter_definition_parameter_id")
                        .from("part_parameter_definition", "parameter_id")
                        .to("parameter", "id")
                        .on_delete(ForeignKeyAction::Restrict)  // need confirmation or shouldn't be able to delete param that is used
                    )
                    .foreign_key(ForeignKey::create()
                        .name("fk_part_parameter_definition_part_id")
                        .from("part_parameter_definition", "part_id")
                        .to("part", "id")
                        .on_delete(ForeignKeyAction::Cascade)
                    )
                    .to_owned(),
            )
            .await?;

        _manager
            .create_table(
                Table::create()
                    .table("part_parameter_value")
                    .if_not_exists()
                    .col(ColumnDef::new("parameter_id").integer().not_null())
                    .col(ColumnDef::new("part_id").integer().not_null())
                    .col(ColumnDef::new("value_real").double())
                    .col(ColumnDef::new("value_string").string())
                    .col(ColumnDef::new("value_int").integer())
                    .primary_key(Index::create()
                        .col("parameter_id")
                        .col("part_id")
                    )
                    .foreign_key(ForeignKey::create()
                        .name("fk_part_parameter_value_parameter_id")
                        .from("part_parameter_value", "parameter_id")
                        .to("parameter", "id")
                        .on_delete(ForeignKeyAction::Restrict)  // need confirmation or shouldn't be able to delete param that is used
                    )
                    .foreign_key(ForeignKey::create()
                        .name("fk_part_parameter_value_part_id")
                        .from("part_parameter_value", "part_id")
                        .to("part", "id")
                        .on_delete(ForeignKeyAction::Cascade)
                    )
                    .to_owned(),
            )
            .await?;

        _manager
            .create_table(
                Table::create()
                    .table("legal_entity")
                    .if_not_exists()
                    .col(pk_auto("id"))
                    .col(ColumnDef::new("owner_id").integer())
                    .col(ColumnDef::new("name").string().not_null())
                    .col(ColumnDef::new("url").string())
                    .col(ColumnDef::new("supplier").boolean().not_null().default(false))
                    .col(ColumnDef::new("manufacturer").boolean().not_null().default(false))
                    .col(ColumnDef::new("customer").boolean().not_null().default(false))
                    .col(ColumnDef::new("owner").boolean().not_null().default(false))
                    .foreign_key(ForeignKey::create()
                        .name("fk_legal_entity_owner_id")
                        .from("legal_entity", "owner_id")
                        .to("legal_entity", "id")
                        .on_delete(ForeignKeyAction::SetNull)
                    )
                    .to_owned(),
            )
            .await?;

        Ok(())
    }

    async fn down(&self, _manager: &SchemaManager) -> Result<(), DbErr> {
        _manager
            .drop_table(Table::drop().table("part").to_owned())
            .await?;
        _manager
            .drop_table(Table::drop().table("item").to_owned())
            .await?;
        _manager
            .drop_table(Table::drop().table("location").to_owned())
            .await?;
        _manager
            .drop_table(Table::drop().table("category").to_owned())
            .await?;
        _manager
            .drop_table(Table::drop().table("parameter").to_owned())
            .await?;
        _manager
            .drop_table(Table::drop().table("part_parameter_definition").to_owned())
            .await?;
        _manager
            .drop_table(Table::drop().table("part_parameter_value").to_owned())
            .await?;
        _manager
            .drop_table(Table::drop().table("legal_entity").to_owned())
            .await?;

        Ok(())
    }
}
