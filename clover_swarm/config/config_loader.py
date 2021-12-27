import enum
import math
from distutils.util import strtobool
from typing import Any, Dict, List, Optional, Union
from xml.etree import ElementTree

import attr
import cattr
import yaml
from cattr.gen import make_dict_structure_fn, make_dict_unstructure_fn, override
from cattr.preconf.pyyaml import make_converter

value_factory = attr.Factory(lambda self: self.default, takes_self=True)


def converter(attribute):
    def decorator(func):
        attribute.converter = func
        return func

    return decorator


@attr.define(kw_only=True, on_setattr=[attr.setters.convert, attr.setters.validate])
class Option:
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
class BoolOption(Option):
    default: bool = attr.field(default=False)
    value: bool = attr.field(init=False, default=value_factory, validator=attr.validators.instance_of(bool))

    @staticmethod
    @converter(value)
    def convert(value):
        if isinstance(value, str):
            value = bool(strtobool(value))
        return value


# fixme: Raises unclear errors
@attr.define(kw_only=True, on_setattr=[attr.setters.convert, attr.setters.validate])
class StrOption(Option):
    default: str = attr.field(default=False)
    value: str = attr.field(init=False, default=value_factory, validator=attr.validators.instance_of(str))

    #
    # @value.validator
    # def validate(self, attribute, value):
    #     # attr.validators.instance_of(str)(self, attribute, value)
    #     if value is None:
    #         raise ValueError
    #     if value == "":
    #         raise ValueError("Attribute must not be empty string!!!!")


@attr.define(kw_only=True, on_setattr=[attr.setters.convert, attr.setters.validate])
class NumericOption(Option):
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
class IntOption(NumericOption):
    option_type = int
    default: Optional[option_type] = attr.field(default=float("nan"))
    value: option_type = attr.field(init=False, default=value_factory, converter=option_type)


@attr.define(kw_only=True)
class EnumItem:
    name: str = attr.field(default=None)
    description: Optional[str] = attr.field(default=None)


@attr.define(kw_only=True, on_setattr=[attr.setters.convert, attr.setters.validate])
class EnumOption(Option):
    items: Dict[str, EnumItem] = attr.field(factory=dict)
    strict: bool = attr.field(default=True)

    value: str = attr.field(init=False, default=value_factory)

    # @value.validator
    # def validate(self, attribute, value):

    def __attrs_post_init__(self):
        for key, item in self.items.items():
            item.name = key


OPTION_TYPES = Union[Option, BoolOption, EnumOption, StrOption, NumericOption, IntOption]


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

    converter: cattr.GenConverter = attr.field(factory=make_converter)

    def __attrs_post_init__(self):
        for key, option in self.options.items():
            option.name = key
            # option.config = self

    def import_config(self, fd):
        ...

    def export_config(self, fd):
        ...


# def bool_to_xml_str(value: bool, cls):
#     return str(value).lower()


@attr.define(kw_only=True)
class XMLConfig(Config):
    tree: ElementTree = attr.field(init=False)

    # def _get_node(self, root):

    def __attrs_post_init__(self):
        super().__attrs_post_init__()
        self._register_hooks()

    def _register_hooks(self):
        self.converter.register_unstructure_hook(bool, lambda value: str(value).lower())

    def import_config(self, fd):
        parser = ElementTree.XMLParser(target=ElementTree.TreeBuilder(insert_comments=True))
        self.tree = ElementTree.parse(fd, parser=parser)
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

            value = str(self.converter.unstructure(option.get()))
            value_action = option.tree_path[1]
            if value_action == "attrib":
                attrib_name = option.tree_path[2]
                node.set(attrib_name, value)
            elif value_action == "text":
                node.text = value
            else:
                raise ValueError

        self.tree.write(fd)  # todo preserve formatting?


CONFIG_TYPES = Union[Config, XMLConfig]


@attr.define()
class ConfigCollection:
    converter: cattr.GenConverter = attr.field(factory=make_converter)
    configs: Dict[str, "Config"] = attr.field(init=False, factory=dict)

    def option_factory(self, o, cls):
        options_mapping = {
            "bool": BoolOption,
            "enum": EnumOption,
            "float": NumericOption,
            "int": IntOption,
            "str": StrOption,
        }

        option_type = options_mapping.get(o["type"], Option)

        obj = self.converter.structure(o, option_type)
        return obj

    def config_factory(self, o, cls):
        configs_mapping = {
            "xml": XMLConfig,
        }

        config_type = configs_mapping.get(o["type"])
        obj = self.converter.structure(o, config_type)
        return obj

    def ignore_no_init(self, cls):
        overrides = {a.name: override(omit=True) for a in attr.fields(cls) if not a.init}
        return make_dict_structure_fn(cls, self.converter, **overrides)

    def __attrs_post_init__(self):
        self._register_hooks()

    def _register_hooks(self):
        self.converter.register_structure_hook(OPTION_TYPES, self.option_factory)
        self.converter.register_structure_hook(CONFIG_TYPES, self.config_factory)

        # Apply to all attr classes
        self.converter.register_structure_hook_factory(attr.has, self.ignore_no_init)

    def load_descriptor(self, fd, import_config=True):
        with open(p) as f:
            descriptor_unstructured = yaml.safe_load(f)

        config = self.converter.structure(descriptor_unstructured, CONFIG_TYPES)
        self.configs[config.name] = config

        if import_config:
            config.import_config(config.path)

        return config

    def load_descriptors(self, path):
        # load folder
        pass


if __name__ == "__main__":
    import os

    p = os.path.abspath("descriptions/aruco_launch.yaml")
    # p = os.path.abspath('descriptions/led_launch.yaml')

    collection = ConfigCollection()
    collection.load_descriptor(p)

    for config in collection.configs.values():
        print(config.name)
        print(config)

        for option in config.options.values():
            print(option.name, option.value, type(option.value))

        config.export_config("configs/led_export.launch")

    # config.options["aruco_detect"].set(True)
    # config.options["gpio_pin"].value = "1243"
    # print(config.options["gpio_pin"].value)

    # with open("configs/aruco_export.launch", "w") as f:
