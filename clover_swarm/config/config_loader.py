import yaml
import attr
import cattr
from cattr.gen import make_dict_unstructure_fn, make_dict_structure_fn, override
from cattr.preconf.pyyaml import make_converter

import enum
from typing import List, Dict, Optional, Any, Union


@attr.define(kw_only=True)
class ConfigOption:
    # config = attr.field(init=False, default=None)
    name: str = attr.field(init=False, default=None)
    tree_path: List[str] = attr.field(default=attr.Factory(lambda self: [self.name], takes_self=True))
    type: str = attr.field()
    default: Optional[Any] = attr.field(default=None)
    description: Optional[str] = attr.field(default=None)

    value: Any = attr.field(init=False, default=attr.Factory(lambda self: self.default, takes_self=True))

    @value.validator
    def validate(self, attribute, value):
        if value is None:
            return

        if self.type == "bool":
            if not isinstance(value, bool):
                raise ValueError(f"NOT bool {value} {attribute}")

    def set(self, new_value):
        self.value = new_value

    def get(self):
        return self.value


@attr.define(kw_only=True)
class EnumItem:
    name: str = attr.field(default=None)
    description: Optional[str] = attr.field(default=None)


@attr.define(kw_only=True)
class ConfigOptionEnum(ConfigOption):
    items: Dict[str, EnumItem] = attr.field(factory=dict)
    strict: bool = attr.field(default=True)

    def __attrs_post_init__(self):
        for key, item in self.items.items():
            item.name = key


CONFIG_OPTIONS = Union[ConfigOption, ConfigOptionEnum]


class ConfigTypes(enum.Enum):
    XML = "xml"
    TOML = "toml"

    LAUNCH = XML


@attr.define(kw_only=True)
class Config:
    path: str = attr.field()
    type: ConfigTypes = attr.field()
    name: str = attr.field()
    description: Optional[str] = attr.field()
    options: Dict[str, CONFIG_OPTIONS] = attr.field(factory=dict)

    def __attrs_post_init__(self):
        for key, option in self.options.items():
            option.name = key
            # option.config = self


if __name__ == '__main__':
    import os
    p = os.path.abspath('descriptions/aruco_launch.yaml')

    with open(p) as f:
        aruco_descriptor = yaml.safe_load(f)

    print(aruco_descriptor)

    converter = make_converter()

    def option_factory(o, cls):
        options_mapping = {
            "enum": ConfigOptionEnum,
        }

        option_type = options_mapping.get(o["type"], ConfigOption)

        obj = converter.structure(o, option_type)
        return obj

    converter.register_structure_hook(CONFIG_OPTIONS, option_factory)

    config = converter.structure(aruco_descriptor, Config)

    print(config)
    config.options["aruco_detect"].set(True)
    print(config.options["aruco_detect"].value)
    config.options["aruco_detect"].set("h")

