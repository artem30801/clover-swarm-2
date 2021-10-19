import yaml
import attr
import cattr
from cattr.gen import make_dict_unstructure_fn, make_dict_structure_fn, override
from cattr.preconf.pyyaml import make_converter
from xml.etree import ElementTree

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
    # config: "Config" = attr.field(init=False, default=None)
    # _export_ref: Optional[Any] = attr.field(init=False)

    name: str = attr.field(init=False, default=None)
    tree_path: List = attr.field(default=attr.Factory(lambda self: [self.name], takes_self=True))
    default: Optional[Any] = attr.field(default=None)
    description: Optional[str] = attr.field(default=None)

    value: Any = attr.field(init=False, default=value_factory)

    @value.validator
    def validate(self, attribute, value):
        if value is None:
            return

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


# fixme: Raises unclear errors
@attr.define(kw_only=True, on_setattr=[attr.setters.convert, attr.setters.validate])
class StrConfigOption(ConfigOption):
    default: str = attr.field(default=False)
    value: str = attr.field(init=False, default=value_factory,
                            validator=attr.validators.instance_of(str))

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
    option_type = float
    allow_nan: bool = attr.field(default=False)  # I think, it's good idea to prohibit nan by default
    lt: Optional[option_type] = attr.field(default=None)
    gt: Optional[option_type] = attr.field(default=None)
    lte: Optional[option_type] = attr.field(default=None)
    gte: Optional[option_type] = attr.field(default=None)

    default: Optional[option_type] = attr.field(default=float("nan"))
    value: option_type = attr.field(init=False, default=value_factory, converter=option_type)

    @value.validator
    def validate(self, attribute, value):
        if not self.allow_nan and math.isnan(value):
            raise ValueError(f"Argument {self.name} must not be nan!")

        if self.lt is not None and value >= self.lt:
            raise ValueError(f"Argument {self.name} must be less than {self.lt}, got {value}")

        if self.gt is not None and value <= self.gt:
            raise ValueError(f"Argument {self.name} must be greater than {self.gt}, got {value}")

        if self.lte is not None and value > self.lte:
            raise ValueError(f"Argument {self.name} must be less or equal to {self.lte}, got {value}")

        if self.gte is not None and value < self.gte:
            raise ValueError(f"Argument {self.name} must be greater or equal to{self.qte}, got {value}")


@attr.define(kw_only=True, on_setattr=[attr.setters.convert, attr.setters.validate])
class IntConfigOption(NumericConfigOption):
    option_type = int
    default: Optional[option_type] = attr.field(default=float("nan"))
    value: option_type = attr.field(init=False, default=value_factory, converter=option_type)


@attr.define(kw_only=True)
class EnumItem:
    name: str = attr.field(default=None)
    description: Optional[str] = attr.field(default=None)


@attr.define(kw_only=True, on_setattr=[attr.setters.convert, attr.setters.validate])
class ConfigOptionEnum(ConfigOption):
    items: Dict[str, EnumItem] = attr.field(factory=dict)
    strict: bool = attr.field(default=True)

    value: str = attr.field(init=False, default=value_factory)

    # @value.validator
    # def validate(self, attribute, value):


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

    def import_config(self, fd):
        ...

    def export_config(self, fd):
        ...


@attr.define(kw_only=True)
class XMLConfig(Config):
    tree: ElementTree = attr.field(init=False)

    # def _get_node(self, root):

    def import_config(self, fd):
        self.tree = ElementTree.parse(fd)
        root = self.tree.getroot()

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

    def export_config(self, fd):
        root = self.tree.getroot()

        for option in self.options.values():
            tree_path = option.tree_path[0]
            node = root.find(tree_path)

            value = str(option.get())
            value_action = option.tree_path[1]
            if value_action == "attrib":
                attrib_name = option.tree_path[2]
                node.set(attrib_name, value)
            elif value_action == "text":
                node.text = value
            else:
                raise ValueError

        self.tree.write(fd)  # todo preserve comments and formatting


CONFIG_TYPES = Union[Config, XMLConfig]

if __name__ == '__main__':
    import os
    # p = os.path.abspath('descriptions/aruco_launch.yaml')
    p = os.path.abspath('descriptions/aruco_launch.yaml')

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
            "str": StrConfigOption,
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

    def structure(cls):
        overrides = {a.name: override(omit=True) for a in attr.fields(cls) if not a.init}
        return make_dict_structure_fn(cls, converter, **overrides)


    converter.register_structure_hook_factory(attr.has, structure)

    config = converter.structure(aruco_descriptor, CONFIG_TYPES)

    config.import_config(config.path)

    for option in config.options.values():
        print(option.name, option.value, type(option.value))

    config.options["length"].set(1)
    for option in config.options.values():
        print(option.name, option.value, type(option.value))
    # config.options["gpio_pin"].value = "1243"
    # print(config.options["gpio_pin"].value)

    # with open("configs/aruco_export.launch", "w") as f:
    # config.export_config("configs/aruco_export.launch")


# config.options["aruco_detect"].set("h")

