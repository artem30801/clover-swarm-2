import yaml
import attr
import cattr
from cattr.gen import make_dict_unstructure_fn, make_dict_structure_fn, override
from cattr.preconf.pyyaml import make_converter

import math
import enum
from typing import List, Dict, Optional, Any, Union
from distutils.util import strtobool

value_factory = attr.Factory(lambda self: self.default, takes_self=True)


def converter(attribute):
    def decorator(func):
        attribute.converter = func
        return func
    return decorator


@attr.define(kw_only=True, on_setattr=[attr.setters.convert, attr.setters.validate])
class ConfigOption:
    # config = attr.field(init=False, default=None)
    name: str = attr.field(init=False, default=None)
    tree_path: List = attr.field(default=attr.Factory(lambda self: [self.name], takes_self=True))
    # type: str = attr.field()
    default: Optional[Any] = attr.field(default=None)
    description: Optional[str] = attr.field(default=None)

    value: Any = attr.field(init=False, default=value_factory)

    @value.validator
    def validate(self, attribute, value):
        if value is None:
            return

        # if self.type == "bool":
        #     if not isinstance(value, bool):
        #         raise ValueError(f"NOT bool {value} {attribute}")

    def set(self, new_value):
        self.value = new_value

    def get(self):
        return self.value


@attr.define(kw_only=True, on_setattr=[attr.setters.convert, attr.setters.validate])
class BoolConfigOption(ConfigOption):
    default: bool = attr.field(default=False)
    value: bool = attr.field(init=False, default=value_factory,
                             validator=attr.validators.instance_of(bool))

    @staticmethod
    @converter(value)
    def convert(value):
        if isinstance(value, str):
            value = bool(strtobool(value))
        return value

    # @value.validator
    # def validate(self, attribute, value):
    #     attr.validators.instance_of(str)(self, attribute, value)


# fixme: Raises unclear errors
@attr.define(kw_only=True, on_setattr=[attr.setters.convert, attr.setters.validate])
class StrConfigOption(ConfigOption):
    default: str = attr.field(default=False)
    value: str = attr.field(init=False, default=value_factory,
                             validator=attr.validators.instance_of(str))

    # @staticmethod
    # @converter(value)
    # def convert(value):
    #     if not isinstance(value, str):
    #         value = str(value)
    #     return value
    #
    # @value.validator
    # def validate(self, attribute, value):
    #     # attr.validators.instance_of(str)(self, attribute, value)
    #     if value is None:
    #         raise ValueError
    #     if value == "":
    #         raise ValueError("Attribute must not be empty string!!!!")


@attr.define(kw_only=True, on_setattr=[attr.setters.convert, attr.setters.validate])
class NumericConfigOption(ConfigOption):
    type = float
    allow_nan: bool = attr.field(default=False)  # I think, it's good idea to prohibit nan by default
    lt: Optional[type] = attr.field(default=None)
    gt: Optional[type] = attr.field(default=None)
    lte: Optional[type] = attr.field(default=None)
    gte: Optional[type] = attr.field(default=None)

    default: Optional[type] = attr.field(default=float("nan"))
    value: type = attr.field(init=False, default=value_factory)

    @value.validator
    def validate(self, attribute, value):  # todo clear error messages
        if not isinstance(value, self.type):
            try:
                value = self.type(value)
            except ValueError:
                raise TypeError(f"Argument {self.name} must have type of {self.type.__name__}, but got "
                                f"{value.__repr__()}, type: {type(value).__name__}")

        if not self.allow_nan and math.isnan(value):
            raise ValueError(f"Argument {self.name} must not be nan!")

        if self.lt is not None and value >= self.lt:
            raise ValueError

        if self.gt is not None and value <= self.gt:
            raise ValueError

        if self.lte is not None and value > self.lte:
            raise ValueError

        if self.gte is not None and value < self.gte:
            raise ValueError

@attr.define(kw_only=True, on_setattr=[attr.setters.convert, attr.setters.validate])
class IntConfigOption(NumericConfigOption):
    type = int




@attr.define(kw_only=True)
class EnumItem:
    name: str = attr.field(default=None)
    description: Optional[str] = attr.field(default=None)


@attr.define(kw_only=True, on_setattr=[attr.setters.convert, attr.setters.validate])
class ConfigOptionEnum(ConfigOption):
    items: Dict[str, EnumItem] = attr.field(factory=dict)
    strict: bool = attr.field(default=True)

    def __attrs_post_init__(self):
        for key, item in self.items.items():
            item.name = key


OPTION_TYPES = Union[ConfigOption, BoolConfigOption, ConfigOptionEnum, StrConfigOption]


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
    options: Dict[str, OPTION_TYPES] = attr.field(factory=dict)

    def __attrs_post_init__(self):
        for key, option in self.options.items():
            option.name = key
            # option.config = self

    def load_config(self, filename):
        ...


@attr.define(kw_only=True)
class XMLConfig(Config):
    def load_config(self, filename):
        import xml.etree.ElementTree as ET

        tree = ET.parse(filename)
        root = tree.getroot()

        for option in self.options.values():
            tree_path = option.tree_path[0]
            node = root.find(tree_path)
            
            value_action = option.tree_path[1]
            if value_action == "attrib":
                attrib_name = option.tree_path[2]
                value = node.get(attrib_name)
            elif value_action == "text":
                value = node.text
            else:
                raise ValueError

            option.set(value)


CONFIG_TYPES = Union[Config, XMLConfig]

if __name__ == '__main__':
    import os
    p = os.path.abspath('descriptions/led_launch.yaml')

    with open(p) as f:
        aruco_descriptor = yaml.safe_load(f)

    print(aruco_descriptor)

    converter = make_converter()

    def option_factory(o, cls):
        options_mapping = {
            "bool": BoolConfigOption,
            "enum": ConfigOptionEnum,
            "float": NumericConfigOption,
            "int": IntConfigOption,
            "str": StrConfigOption
        }

        option_type = options_mapping.get(o["type"], ConfigOption)

        obj = converter.structure(o, option_type)
        return obj

    def config_factory(o, cls):
        configs_mapping = {
            "xml": XMLConfig
        }

        config_type = configs_mapping.get(o["type"])
        obj = converter.structure(o, config_type)
        return obj

    converter.register_structure_hook(OPTION_TYPES, option_factory)
    converter.register_structure_hook(CONFIG_TYPES, config_factory)

    config = converter.structure(aruco_descriptor, CONFIG_TYPES)

    print(config)

    config.load_config("configs/led.launch")

    # config.options["ws281x"].set(True)
    print(config.options["led_count"].value)
    config.options["led_count"].set(1.0)
    print(config.options["led_count"].value)
    print(type(config.options["led_count"].value))  # fixme: int value expected!!!!!
    # print(config.options["map"].value)
    # config.options["aruco_detect"].set("h")

