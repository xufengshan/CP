def get_selected_car_platform(name: str):
  from opendbc.car.ford.values import CAR as FORD
  from opendbc.car.hyundai.values import CAR as HYUNDAI
  from opendbc.car.gm.values import CAR as GM
  from opendbc.car.toyota.values import CAR as TOYOTA
  from opendbc.car.mazda.values import CAR as MAZDA
  from opendbc.car.volkswagen.values import CAR as VOLKSWAGEN
  from opendbc.car.tesla.values import CAR as TESLA
  from opendbc.car.byd.values import CAR as BYD

  platforms = [platform for brand in (FORD, GM, TOYOTA, HYUNDAI, MAZDA, VOLKSWAGEN, BYD) for platform in brand]
  # Model X is intentionally dashcam-only. Model 3/Y have a CarController and
  # must be selectable even when automatic fingerprinting is unavailable.
  platforms.extend((TESLA.TESLA_MODEL_3, TESLA.TESLA_MODEL_Y))

  # 归一化 + 多候选匹配 (学习硬件 cp11 明文 selected_car.py 的归一化思路, 适配 VM doc.name 格式)
  # 兼容: 枚举名(BYD_TANG_DM) / 平台名 / 显示名(BYD TANG DM) / 带厂商前缀名(Byd Tang DM→去前缀)
  #       大小写不敏感 (byd tang dm / Byd Tang DM), 并对 doc.name 也做去厂商前缀归一化
  normalized_name = name.strip()
  candidates = {normalized_name.lower()}
  if " " in normalized_name:
    candidates.add(normalized_name.split(" ", 1)[1].strip().lower())

  def doc_matches(platform):
    # 枚举名/平台名精确匹配 (大小写不敏感)
    if normalized_name.lower() in (str(platform).lower(), platform.name.lower()):
      return True
    # 显示名匹配: doc.name 也去厂商前缀后做交集
    for doc in platform.config.car_docs:
      dn = doc.name.strip().lower()
      doc_cands = {dn}
      if " " in dn:
        doc_cands.add(dn.split(" ", 1)[1].strip())
      if candidates & doc_cands:
        return True
    return False

  return next((platform for platform in platforms if doc_matches(platform)), None)
